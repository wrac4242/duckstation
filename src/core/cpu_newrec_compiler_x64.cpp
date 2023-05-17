// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "cpu_newrec_compiler_x64.h"
#include "common/align.h"
#include "common/assert.h"
#include "common/log.h"
#include "common/string_util.h"
#include "cpu_core_private.h"
#include "cpu_newrec_private.h"
#include "cpu_recompiler_thunks.h"
#include "settings.h"
#include "timing_event.h"
Log_SetChannel(CPU::NewRec);

#define DUMP_BLOCKS

#ifdef DUMP_BLOCKS
#include "Zycore/Format.h"
#include "Zycore/Status.h"
#include "Zydis/Zydis.h"
#endif

#define PTR(x) (cg->rip + (x))
// #define PTR(x) (cg->rbp + (u32)(((u8*)(x)) - ((u8*)&g_state)))

#if defined(_WIN32)
#define RWRET cg->eax
#define RWARG1 cg->ecx
#define RWARG2 cg->edx
#define RWARG3 cg->r8d
#define RXRET cg->rax
#define RXARG1 cg->rcx
#define RXARG2 cg->rdx
#define RXARG3 cg->r8
#else
#error fixme
#endif

using namespace Xbyak;

static bool IsCallerSavedRegister(int id)
{
#ifdef _WIN32
  // The x64 ABI considers the registers RAX, RCX, RDX, R8, R9, R10, R11, and XMM0-XMM5 volatile.
  return (id <= 2 || (id >= 8 && id <= 11));
#else
  // rax, rdi, rsi, rdx, rcx, r8, r9, r10, r11 are scratch registers.
  return (id <= 2 || id == 6 || id == 7 || (id >= 8 && id <= 11));
#endif
}

namespace CPU::CodeCache {
void LogCurrentState();
}

CPU::NewRec::X64Compiler::X64Compiler() = default;

CPU::NewRec::X64Compiler::~X64Compiler() = default;

void CPU::NewRec::X64Compiler::Reset(Block* block)
{
  Compiler::Reset(block);

  if (cg)
    cg.reset();

  cg = std::make_unique<Xbyak::CodeGenerator>(g_code_buffer.GetFreeCodeSpace(), g_code_buffer.GetFreeCodePointer());

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    if (i == static_cast<u32>(RWRET.getIdx()) || i == static_cast<u32>(RWARG1.getIdx()) ||
        i == static_cast<u32>(RWARG2.getIdx()) || i == static_cast<u32>(RWARG3.getIdx()) ||
        i == static_cast<u32>(cg->rsp.getIdx()) ||
        i == static_cast<u32>(cg->ecx.getIdx()) /* keep ecx free for shifts, maybe use BMI? */)
    {
      continue;
    }

    HostRegAlloc& ra = m_host_regs[i];
    ra.flags = HR_USABLE | (IsCallerSavedRegister(i) ? 0 : HR_CALLEE_SAVED);
  }
}

void CPU::NewRec::X64Compiler::BeginBlock()
{
  Compiler::BeginBlock();
#if 1
  cg->call(&CPU::CodeCache::LogCurrentState);
#endif

#if 0
  if (m_block->pc == 0x8001D644)
  {
    //__debugbreak();
    cg->db(0xcc);
  }
#endif
}

void CPU::NewRec::X64Compiler::EndBlock(std::optional<u32> newpc)
{
  const bool link_block = false || !newpc.has_value();

  if (newpc.has_value())
  {
    if (m_dirty_pc || m_compiler_pc != newpc)
      cg->mov(cg->dword[PTR(&g_state.regs.pc)], newpc.value());
  }
  m_dirty_pc = false;

  // save cycles for event test
  const TickCount cycles = std::exchange(m_cycles, 0);

  // flush regs
  Flush(FLUSH_END_BLOCK);

  // event test
  // pending_ticks += cycles
  // if (pending_ticks >= downcount) { dispatch_event(); }
  cg->mov(RWARG1, cg->dword[PTR(&g_state.pending_ticks)]);
  if (cycles > 0)
    cg->add(RWARG1, cycles);
  cg->cmp(RWARG1, cg->dword[PTR(&g_state.downcount)]);
  if (cycles > 0)
    cg->mov(cg->dword[PTR(&g_state.pending_ticks)], RWARG1);
  cg->jge(g_event_test_and_dispatch);

  // jump to dispatcher
  // TODO: link here
  cg->jmp(g_dispatcher);

  m_block_ended = true;
}

void CPU::NewRec::X64Compiler::EndBlockWithException(Exception excode)
{
  // flush regs, but not pc, it's going to get overwritten
  Flush(FLUSH_END_BLOCK);

  // TODO: flush load delay
  // TODO: break for pcdrv

  cg->mov(RWARG1, Cop0Registers::CAUSE::MakeValueForException(excode, m_current_instruction_branch_delay_slot, false,
                                                              inst->cop.cop_n));
  cg->mov(RWARG2, m_current_instruction_pc);
  cg->call(static_cast<void (*)(u32, u32)>(&CPU::RaiseException));
  cg->jmp(g_event_test_and_dispatch);
  m_block_ended = true;
}

std::pair<const void*, u32> CPU::NewRec::X64Compiler::EndCompile()
{
  return std::pair<const void*, u32>(cg->getCode(), static_cast<u32>(cg->getSize()));
}

const void* CPU::NewRec::X64Compiler::GetCurrentCodePointer()
{
  return cg->getCurr();
}

#ifdef DUMP_BLOCKS
static ZydisFormatterFunc s_old_print_address;

static ZyanStatus ZydisFormatterPrintAddressAbsolute(const ZydisFormatter* formatter, ZydisFormatterBuffer* buffer,
                                                     ZydisFormatterContext* context)
{
  using namespace CPU;

  ZyanU64 address;
  ZYAN_CHECK(ZydisCalcAbsoluteAddress(context->instruction, context->operand, context->runtime_address, &address));

  char buf[128];
  u32 len = 0;

#define A(x) static_cast<ZyanU64>(reinterpret_cast<uintptr_t>(x))

  if (address >= A(Bus::g_ram) && address < A(Bus::g_ram + Bus::g_ram_size))
  {
    len = snprintf(buf, sizeof(buf), "g_ram+0x%08X", static_cast<u32>(address - A(Bus::g_ram)));
  }
  else if (address >= A(&g_state.regs) &&
           address < A(reinterpret_cast<const u8*>(&g_state.regs) + sizeof(CPU::Registers)))
  {
    len = snprintf(buf, sizeof(buf), "g_state.regs.%s",
                   GetRegName(static_cast<CPU::Reg>(((address - A(&g_state.regs.r[0])) / 4u))));
  }
  else if (address >= A(&g_state.cop0_regs) &&
           address < A(reinterpret_cast<const u8*>(&g_state.cop0_regs) + sizeof(CPU::Cop0Registers)))
  {
    for (const DebuggerRegisterListEntry& rle : g_debugger_register_list)
    {
      if (address == static_cast<ZyanU64>(reinterpret_cast<uintptr_t>(rle.value_ptr)))
      {
        len = snprintf(buf, sizeof(buf), "g_state.cop0_regs.%s", rle.name);
        break;
      }
    }
  }
  else if (address >= A(&g_state.gte_regs) &&
           address < A(reinterpret_cast<const u8*>(&g_state.gte_regs) + sizeof(GTE::Regs)))
  {
    for (const DebuggerRegisterListEntry& rle : g_debugger_register_list)
    {
      if (address == static_cast<ZyanU64>(reinterpret_cast<uintptr_t>(rle.value_ptr)))
      {
        len = snprintf(buf, sizeof(buf), "g_state.gte_regs.%s", rle.name);
        break;
      }
    }
  }
  else if (address == A(&g_state.pending_ticks))
  {
    len = snprintf(buf, sizeof(buf), "g_state.pending_ticks");
  }
  else if (address == A(&g_state.downcount))
  {
    len = snprintf(buf, sizeof(buf), "g_state.downcount");
  }

#undef A

  if (len > 0)
  {
    ZYAN_CHECK(ZydisFormatterBufferAppend(buffer, ZYDIS_TOKEN_SYMBOL));
    ZyanString* string;
    ZYAN_CHECK(ZydisFormatterBufferGetString(buffer, &string));
    return ZyanStringAppendFormat(string, "&%s", buf);
  }

  return s_old_print_address(formatter, buffer, context);
}
#endif

void CPU::NewRec::X64Compiler::DisassembleAndLog(const void* start, u32 size)
{
  ZydisDecoder disas_decoder;
  ZydisFormatter disas_formatter;
  ZydisDecodedInstruction disas_instruction;
  ZydisDecodedOperand disas_operands[ZYDIS_MAX_OPERAND_COUNT];
  ZydisDecoderInit(&disas_decoder, ZYDIS_MACHINE_MODE_LONG_64, ZYDIS_STACK_WIDTH_64);
  ZydisFormatterInit(&disas_formatter, ZYDIS_FORMATTER_STYLE_INTEL);
  s_old_print_address = (ZydisFormatterFunc)&ZydisFormatterPrintAddressAbsolute;
  ZydisFormatterSetHook(&disas_formatter, ZYDIS_FORMATTER_FUNC_PRINT_ADDRESS_ABS, (const void**)&s_old_print_address);

  const u8* instPtr = static_cast<const u8*>(start);
  ZyanUSize instLength = size;
  while (ZYAN_SUCCESS(ZydisDecoderDecodeFull(&disas_decoder, instPtr, instLength, &disas_instruction, disas_operands)))
  {
    char buffer[256];
    if (ZYAN_SUCCESS(ZydisFormatterFormatInstruction(
          &disas_formatter, &disas_instruction, disas_operands, ZYDIS_MAX_OPERAND_COUNT, buffer, sizeof(buffer),
          static_cast<ZyanU64>(reinterpret_cast<uintptr_t>(instPtr)), nullptr)))
    {
      Log_DebugPrintf("    %016" PRIX64 "    %s", static_cast<u64>(reinterpret_cast<uintptr_t>(instPtr)), buffer);
    }

    instPtr += disas_instruction.length;
    instLength -= disas_instruction.length;
  }
#if 0
  const std::string str = StringUtil::EncodeHex(static_cast<const u8*>(start), size);
  Log_DebugPrint(str.c_str());
#endif
}

void CPU::NewRec::X64Compiler::FlushConstantReg(Reg r)
{
  cg->mov(cg->dword[PTR(&g_state.regs.r[static_cast<u32>(r)])], m_constant_reg_values[static_cast<u32>(r)]);
  Compiler::FlushConstantReg(r);
}

void CPU::NewRec::X64Compiler::PopulateHostReg(u32 reg)
{
  DebugAssert(reg < NUM_HOST_REGS);
  HostRegAlloc& ra = m_host_regs[reg];
  switch (ra.type)
  {
    case HR_TYPE_CPU_REG:
    {
      DebugAssert(ra.reg >= 0 && ra.reg < static_cast<s8>(Reg::count));
      if (HasConstantReg(static_cast<Reg>(ra.reg)))
        cg->mov(Reg32(reg), GetConstantRegU32(static_cast<Reg>(ra.reg)));
      else
        cg->mov(Reg32(reg), cg->dword[PTR(&g_state.regs.r[ra.reg])]);
    }
    break;

    case HR_TYPE_LOAD_DELAY_VALUE:
      cg->mov(Reg32(reg), cg->dword[PTR(&g_state.load_delay_value)]);
      break;

    case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
      cg->mov(Reg32(reg), cg->dword[PTR(&g_state.next_load_delay_value)]);
      break;

    default:
      Panic("Invalid reg type");
      break;
  }
}

void CPU::NewRec::X64Compiler::WritebackHostReg(u32 reg)
{
  DebugAssert(reg < NUM_HOST_REGS);
  HostRegAlloc& ra = m_host_regs[reg];
  switch (ra.type)
  {
    case HR_TYPE_CPU_REG:
      cg->mov(cg->dword[PTR(&g_state.regs.r[ra.reg])], Reg32(reg));
      break;

    case HR_TYPE_LOAD_DELAY_VALUE:
      cg->mov(cg->dword[PTR(&g_state.load_delay_value)], Reg32(reg));
      break;

    case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
      cg->mov(cg->dword[PTR(&g_state.next_load_delay_value)], Reg32(reg));
      break;

    default:
      Panic("Invalid reg type");
      break;
  }
}

void CPU::NewRec::X64Compiler::CopyHostReg(u32 dst, u32 src)
{
  if (src != dst)
    cg->mov(Reg32(dst), Reg32(src));
}

Xbyak::Address CPU::NewRec::X64Compiler::MipsPtr(Reg r) const
{
  DebugAssert(r < Reg::count);
  return cg->dword[PTR(&g_state.regs.r[static_cast<u32>(r)])];
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::CFGetRegD(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_d);
  return Reg32(cf.host_d);
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::CFGetRegS(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_s);
  return Reg32(cf.host_s);
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::CFGetRegT(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_t);
  return Reg32(cf.host_t);
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::CFGetRegLO(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_lo);
  return Reg32(cf.host_lo);
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::CFGetRegHI(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_hi);
  return Reg32(cf.host_hi);
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::MoveSToD(CompileFlags cf)
{
  DebugAssert(cf.valid_host_d);
  DebugAssert(!cf.valid_host_t || cf.host_t != cf.host_d);

  const Reg32 rd = CFGetRegD(cf);
#if 0
  if (cf.valid_host_s)
  {
    const Reg32 rs = CFGetRegS(cf);
    if (rd != rs)
      cg->mov(rd, CFGetRegS(cf));
  }
  else if (cf.const_s)
  {
    if (const u32 cv = GetConstantRegU32(cf.MipsS()); cv != 0)
      cg->mov(rd, GetConstantRegU32(cf.MipsS()));
    else
      cg->xor_(rd, rd);
  }
  else
  {
    cg->mov(rd, MipsPtr(cf.MipsS()));
  }
#else
  MoveSToReg(rd, cf);
#endif

  return rd;
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::MoveSToT(CompileFlags cf)
{
  DebugAssert(cf.valid_host_t);

  const Reg32 rt = CFGetRegT(cf);
  if (cf.valid_host_s)
  {
    const Reg32 rs = CFGetRegS(cf);
    if (rt != rs)
      cg->mov(rt, rs);
  }
  else if (cf.const_s)
  {
    if (const u32 cv = GetConstantRegU32(cf.MipsS()); cv != 0)
      cg->mov(rt, cv);
    else
      cg->xor_(rt, rt);
  }
  else
  {
    cg->mov(rt, MipsPtr(cf.MipsS()));
  }

  return rt;
}

Xbyak::Reg32 CPU::NewRec::X64Compiler::MoveTToD(CompileFlags cf)
{
  DebugAssert(cf.valid_host_d);
  DebugAssert(!cf.valid_host_s || cf.host_s != cf.host_d);

  const Reg32 rd = CFGetRegD(cf);
  MoveTToReg(rd, cf);
  return rd;
}

void CPU::NewRec::X64Compiler::MoveSToReg(const Xbyak::Reg32& dst, CompileFlags cf)
{
  if (cf.valid_host_s)
  {
    if (cf.host_s != static_cast<u32>(dst.getIdx()))
      cg->mov(dst, Reg32(cf.host_s));
  }
  else if (cf.const_s)
  {
    const u32 cv = GetConstantRegU32(cf.MipsS());
    if (cv == 0)
      cg->xor_(dst, dst);
    else
      cg->mov(dst, cv);
  }
  else
  {
    cg->mov(dst, cg->dword[PTR(&g_state.regs.r[cf.mips_s])]);
  }
}

void CPU::NewRec::X64Compiler::MoveTToReg(const Xbyak::Reg32& dst, CompileFlags cf)
{
  if (cf.valid_host_t)
  {
    if (cf.host_t != static_cast<u32>(dst.getIdx()))
      cg->mov(dst, Reg32(cf.host_t));
  }
  else if (cf.const_t)
  {
    const u32 cv = GetConstantRegU32(cf.MipsT());
    if (cv == 0)
      cg->xor_(dst, dst);
    else
      cg->mov(dst, cv);
  }
  else
  {
    cg->mov(dst, cg->dword[PTR(&g_state.regs.r[cf.mips_t])]);
  }
}

void CPU::NewRec::X64Compiler::Flush(u32 flags)
{
  Compiler::Flush(flags);

  if (flags & FLUSH_PC && m_dirty_pc)
  {
    cg->mov(cg->dword[PTR(&g_state.regs.pc)], m_compiler_pc);
    m_dirty_pc = false;
  }

  if (flags & FLUSH_INSTRUCTION_BITS)
  {
    cg->mov(cg->dword[PTR(&g_state.current_instruction.bits)], inst->bits);
    cg->mov(cg->dword[PTR(&g_state.current_instruction_pc)], m_current_instruction_pc);
    cg->mov(cg->byte[PTR(&g_state.current_instruction_in_branch_delay_slot)], m_current_instruction_branch_delay_slot);
  }

  if (flags & FLUSH_CYCLES && m_cycles > 0)
  {
    cg->add(cg->dword[PTR(&g_state.pending_ticks)], m_cycles);
    m_cycles = 0;
  }

  if (flags & FLUSH_LOAD_DELAY_FROM_STATE && m_load_delay_dirty)
  {
    // This sucks :(
    Label no_load_delay;
    cg->movzx(RWARG1, cg->byte[PTR(&g_state.load_delay_reg)]);
    cg->cmp(RWARG1, static_cast<u8>(Reg::count));
    cg->je(no_load_delay);
    cg->mov(RWARG2, cg->dword[PTR(&g_state.load_delay_value)]);
    cg->lea(RXARG3, cg->dword[PTR(&g_state.regs.r[0])]);
    cg->mov(cg->dword[RXARG3 + RXARG1 * 4], RWARG2);
    cg->mov(cg->byte[PTR(&g_state.load_delay_reg)], static_cast<u8>(Reg::count));
    cg->L(no_load_delay);
    m_load_delay_dirty = false;
  }

  if (flags & FLUSH_LOAD_DELAY && m_load_delay_register != Reg::count)
  {
    if (m_load_delay_value_register != NUM_HOST_REGS)
      FreeHostReg(m_load_delay_value_register);

    cg->mov(cg->byte[PTR(&g_state.load_delay_reg)], static_cast<u8>(m_load_delay_register));
    m_load_delay_register = Reg::count;
    m_load_delay_dirty = true;
  }
}

void CPU::NewRec::X64Compiler::Compile_Fallback()
{
  Flush(FLUSH_FOR_INTERPRETER);

  cg->call(&CPU::Recompiler::Thunks::InterpretInstruction);

  // TODO: make me less garbage
  // TODO: this is wrong, it flushes the load delay on the same cycle when we return.
  // but nothing should be going through here..
  Label no_load_delay;
  cg->movzx(RWARG1, cg->byte[PTR(&g_state.next_load_delay_reg)]);
  cg->cmp(RWARG1, static_cast<u8>(Reg::count));
  cg->je(no_load_delay, CodeGenerator::T_SHORT);
  cg->mov(RWARG2, cg->dword[PTR(&g_state.next_load_delay_value)]);
  cg->mov(cg->byte[PTR(&g_state.load_delay_reg)], RWARG1);
  cg->mov(cg->dword[PTR(&g_state.load_delay_value)], RWARG2);
  cg->mov(cg->byte[PTR(&g_state.next_load_delay_reg)], static_cast<u32>(Reg::count));
  cg->L(no_load_delay);

  m_load_delay_dirty = true;
}

void CPU::NewRec::X64Compiler::CheckBranchTarget(const Xbyak::Reg32& pcreg)
{
  if (!g_settings.cpu_recompiler_memory_exceptions)
    return;

  // TODO: this gets too large..
  Label aligned;
  cg->test(pcreg, 0x3);
  cg->jz(aligned, CodeGenerator::T_NEAR);

  BackupHostState();
  EndBlockWithException(Exception::AdEL);

  RestoreHostState();
  cg->L(aligned);
}

void CPU::NewRec::X64Compiler::Compile_jr(CompileFlags cf)
{
  if (!cf.valid_host_s)
    cg->mov(RWARG1, MipsPtr(cf.MipsS()));

  const Reg32 pcreg = cf.valid_host_s ? CFGetRegS(cf) : RWARG1;
  CheckBranchTarget(pcreg);

  cg->mov(cg->dword[PTR(&g_state.regs.pc)], pcreg);

  CompileBranchDelaySlot(false);
  EndBlock(std::nullopt);
}

void CPU::NewRec::X64Compiler::Compile_jalr(CompileFlags cf)
{
  if (!cf.valid_host_s)
    cg->mov(RWARG1, MipsPtr(cf.MipsS()));

  const Reg32 pcreg = cf.valid_host_s ? CFGetRegS(cf) : RWARG1;
  CheckBranchTarget(pcreg);

  if (MipsD() != Reg::zero)
  {
    // TODO: Don't allocate here
    if (cf.valid_host_d)
      cg->mov(CFGetRegD(cf), GetBranchReturnAddress());
    else
      cg->mov(MipsPtr(MipsD()), GetBranchReturnAddress());
  }

  cg->mov(cg->dword[PTR(&g_state.regs.pc)], pcreg);
  CompileBranchDelaySlot(false);
  EndBlock(std::nullopt);
}

void CPU::NewRec::X64Compiler::Compile_bxx(CompileFlags cf, BranchCondition cond)
{
  const u32 taken_pc = GetConditionalBranchTarget();

  Flush(FLUSH_FOR_BRANCH);

  DebugAssert(cf.valid_host_s);

  // TODO: Swap this back to near once instructions don't blow up
  constexpr CodeGenerator::LabelType type = CodeGenerator::T_NEAR;
  Label taken;
  switch (cond)
  {
    case BranchCondition::Equal:
    case BranchCondition::NotEqual:
    {
      // we should always have S, maybe not T
      // TODO: if it's zero, we can just do test rs, rs
      if (cf.valid_host_t)
        cg->cmp(CFGetRegS(cf), CFGetRegT(cf));
      else if (cf.const_t)
        cg->cmp(CFGetRegS(cf), GetConstantRegU32(cf.MipsT()));
      else
        cg->cmp(CFGetRegS(cf), MipsPtr(cf.MipsT()));

      (cond == BranchCondition::Equal) ? cg->je(taken, type) : cg->jne(taken, type);
    }
    break;

    case BranchCondition::GreaterThanZero:
    {
      cg->cmp(CFGetRegS(cf), 0);
      cg->jg(taken, type);
    }
    break;

    case BranchCondition::GreaterEqualZero:
    {
      cg->test(CFGetRegS(cf), CFGetRegS(cf));
      cg->jns(taken, type);
    }
    break;

    case BranchCondition::LessThanZero:
    {
      cg->test(CFGetRegS(cf), CFGetRegS(cf));
      cg->js(taken, type);
    }
    break;

    case BranchCondition::LessEqualZero:
    {
      cg->cmp(CFGetRegS(cf), 0);
      cg->jle(taken, type);
    }
    break;
  }

  BackupHostState();
  CompileBranchDelaySlot();
  EndBlock(m_compiler_pc);

  cg->L(taken);

  RestoreHostState();
  CompileBranchDelaySlot();
  EndBlock(taken_pc);
}

void CPU::NewRec::X64Compiler::Compile_addi(CompileFlags cf)
{
  const Reg32 rt = MoveSToT(cf);
  if (const u32 imm = inst->i.imm_sext32(); imm != 0)
  {
    cg->add(rt, imm);
    if (g_settings.cpu_recompiler_memory_exceptions)
    {
      DebugAssert(cf.valid_host_t);
      TestOverflow(rt);
    }
  }
}

void CPU::NewRec::X64Compiler::Compile_addiu(CompileFlags cf)
{
  const Reg32 rt = MoveSToT(cf);
  if (const u32 imm = inst->i.imm_sext32(); imm != 0)
    cg->add(rt, imm);
}

void CPU::NewRec::X64Compiler::Compile_slti(CompileFlags cf)
{
  Compile_slti(cf, true);
}

void CPU::NewRec::X64Compiler::Compile_sltiu(CompileFlags cf)
{
  Compile_slti(cf, false);
}

void CPU::NewRec::X64Compiler::Compile_slti(CompileFlags cf, bool sign)
{
  const Reg32 rt = cf.valid_host_t ? CFGetRegT(cf) : RWARG1;

  // Case where T == S, can't use xor because it changes flags
  if (!cf.valid_host_t || !cf.valid_host_s || cf.host_t != cf.host_s)
    cg->xor_(rt, rt);

  if (cf.valid_host_s)
    cg->cmp(CFGetRegS(cf), inst->i.imm_sext32());
  else
    cg->cmp(MipsPtr(cf.MipsS()), inst->i.imm_sext32());

  if (cf.valid_host_t && cf.valid_host_s && cf.host_t == cf.host_s)
    cg->mov(rt, 0);

  sign ? cg->setl(rt.cvt8()) : cg->setb(rt.cvt8());

  if (!cf.valid_host_t)
    cg->mov(MipsPtr(cf.MipsT()), rt);
}

void CPU::NewRec::X64Compiler::Compile_andi(CompileFlags cf)
{
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
  {
    const Reg32 rt = MoveSToT(cf);
    cg->and_(rt, imm);
  }
  else
  {
    const Reg32 rt = CFGetRegT(cf);
    cg->xor_(rt, rt);
  }
}

void CPU::NewRec::X64Compiler::Compile_ori(CompileFlags cf)
{
  const Reg32 rt = MoveSToT(cf);
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
    cg->or_(rt, imm);
}

void CPU::NewRec::X64Compiler::Compile_xori(CompileFlags cf)
{
  const Reg32 rt = MoveSToT(cf);
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
    cg->xor_(rt, imm);
}

void CPU::NewRec::X64Compiler::Compile_sll(CompileFlags cf)
{
  const Reg32 rd = MoveTToD(cf);
  if (inst->r.shamt > 0)
    cg->shl(rd, inst->r.shamt);
}

void CPU::NewRec::X64Compiler::Compile_srl(CompileFlags cf)
{
  const Reg32 rd = MoveTToD(cf);
  if (inst->r.shamt > 0)
    cg->shr(rd, inst->r.shamt);
}

void CPU::NewRec::X64Compiler::Compile_sra(CompileFlags cf)
{
  const Reg32 rd = MoveTToD(cf);
  if (inst->r.shamt > 0)
    cg->sar(rd, inst->r.shamt);
}

void CPU::NewRec::X64Compiler::Compile_variable_shift(
  CompileFlags cf, void (Xbyak::CodeGenerator::*op)(const Xbyak::Operand&, const Xbyak::Reg8&),
  void (Xbyak::CodeGenerator::*op_const)(const Xbyak::Operand&, int))
{
  const Reg32 rd = CFGetRegD(cf);
  if (!cf.const_s)
  {
    MoveSToReg(cg->ecx, cf);
    MoveTToReg(rd, cf);
    ((cg.get())->*op)(rd, cg->cl);
  }
  else
  {
    MoveTToReg(rd, cf);
    ((cg.get())->*op_const)(rd, GetConstantRegU32(cf.MipsS()));
  }
}

void CPU::NewRec::X64Compiler::Compile_sllv(CompileFlags cf)
{
  Compile_variable_shift(cf, &CodeGenerator::shl, &CodeGenerator::shl);
}

void CPU::NewRec::X64Compiler::Compile_srlv(CompileFlags cf)
{
  Compile_variable_shift(cf, &CodeGenerator::shr, &CodeGenerator::shr);
}

void CPU::NewRec::X64Compiler::Compile_srav(CompileFlags cf)
{
  Compile_variable_shift(cf, &CodeGenerator::sar, &CodeGenerator::sar);
}

void CPU::NewRec::X64Compiler::Compile_mult(CompileFlags cf, bool sign)
{
  // RAX/RDX shouldn't be allocatable..
  DebugAssert(!(m_host_regs[Xbyak::Operand::RAX].flags & HR_USABLE) &&
              !(m_host_regs[Xbyak::Operand::RDX].flags & HR_USABLE));

  MoveSToReg(cg->eax, cf);
  if (cf.valid_host_t)
  {
    sign ? cg->imul(CFGetRegT(cf)) : cg->mul(CFGetRegT(cf));
  }
  else if (cf.const_t)
  {
    cg->mov(cg->edx, GetConstantRegU32(cf.MipsT()));
    sign ? cg->imul(cg->edx) : cg->mul(cg->edx);
  }
  else
  {
    sign ? cg->imul(MipsPtr(cf.MipsT())) : cg->mul(MipsPtr(cf.MipsT()));
  }

  // TODO: skip writeback if it's not needed
  if (cf.valid_host_lo)
    cg->mov(CFGetRegLO(cf), cg->eax);
  else
    cg->mov(MipsPtr(Reg::lo), cg->eax);
  if (cf.valid_host_lo)
    cg->mov(CFGetRegHI(cf), cg->edx);
  else
    cg->mov(MipsPtr(Reg::hi), cg->edx);
}

void CPU::NewRec::X64Compiler::Compile_mult(CompileFlags cf)
{
  Compile_mult(cf, true);
}

void CPU::NewRec::X64Compiler::Compile_multu(CompileFlags cf)
{
  Compile_mult(cf, false);
}

void CPU::NewRec::X64Compiler::TestOverflow(const Xbyak::Reg32& result)
{
  // TODO: far code

  Label no_overflow;
  cg->jno(no_overflow);
  BackupHostState();

  // toss the result
  ClearHostReg(result.getIdx());

  EndBlockWithException(Exception::Ov);

  RestoreHostState();
  cg->L(no_overflow);
}

void CPU::NewRec::X64Compiler::Compile_dst_op(
  CompileFlags cf, void (Xbyak::CodeGenerator::*op)(const Xbyak::Operand&, const Xbyak::Operand&),
  void (Xbyak::CodeGenerator::*op_const)(const Xbyak::Operand&, u32), bool commutative, bool overflow)
{
  if (cf.valid_host_s && cf.valid_host_t)
  {
    if (cf.host_d == cf.host_s)
    {
      ((cg.get())->*op)(CFGetRegD(cf), CFGetRegT(cf));
    }
    else if (cf.host_d == cf.host_t)
    {
      if (commutative)
      {
        ((cg.get())->*op)(CFGetRegD(cf), CFGetRegS(cf));
      }
      else
      {
        cg->mov(RWARG1, CFGetRegT(cf));
        cg->mov(CFGetRegD(cf), CFGetRegS(cf));
        ((cg.get())->*op)(CFGetRegD(cf), RWARG1);
      }
    }
    else
    {
      cg->mov(CFGetRegD(cf), CFGetRegS(cf));
      ((cg.get())->*op)(CFGetRegD(cf), CFGetRegT(cf));
    }
  }
  else if (commutative && (cf.const_s || cf.const_t))
  {
    const Reg32 rd = CFGetRegD(cf);
    (cf.const_s) ? MoveTToReg(rd, cf) : MoveSToReg(rd, cf);
    if (const u32 cv = GetConstantRegU32(cf.const_s ? cf.MipsS() : cf.MipsT()); cv != 0)
      ((cg.get())->*op_const)(CFGetRegD(cf), cv);
    else
      overflow = false;
  }
  else if (cf.const_s)
  {
    // need to backup T?
    if (cf.valid_host_d && cf.valid_host_t && cf.host_d == cf.host_t)
    {
      cg->mov(RWARG1, CFGetRegT(cf));
      MoveSToReg(CFGetRegD(cf), cf);
      ((cg.get())->*op)(CFGetRegD(cf), RWARG1);
    }
    else
    {
      MoveSToReg(CFGetRegD(cf), cf);
      ((cg.get())->*op)(CFGetRegD(cf), CFGetRegT(cf));
    }
  }
  else if (cf.const_t)
  {
    MoveSToReg(CFGetRegD(cf), cf);
    if (const u32 cv = GetConstantRegU32(cf.MipsT()); cv != 0)
      ((cg.get())->*op_const)(CFGetRegD(cf), cv);
    else
      overflow = false;
  }
  else if (cf.valid_host_s)
  {
    if (cf.host_d != cf.host_s)
      cg->mov(CFGetRegD(cf), CFGetRegS(cf));
    ((cg.get())->*op)(CFGetRegD(cf), MipsPtr(cf.MipsT()));
  }
  else if (cf.valid_host_t)
  {
    if (cf.host_d != cf.host_t)
      cg->mov(CFGetRegD(cf), CFGetRegT(cf));
    ((cg.get())->*op)(CFGetRegD(cf), MipsPtr(cf.MipsS()));
  }
  else
  {
    cg->mov(CFGetRegD(cf), MipsPtr(cf.MipsS()));
    ((cg.get())->*op)(CFGetRegD(cf), MipsPtr(cf.MipsT()));
  }

  if (overflow)
  {
    DebugAssert(cf.valid_host_d);
    TestOverflow(CFGetRegD(cf));
  }
}

void CPU::NewRec::X64Compiler::Compile_add(CompileFlags cf)
{
  Compile_dst_op(cf, &CodeGenerator::add, &CodeGenerator::add, true, true);
}

void CPU::NewRec::X64Compiler::Compile_addu(CompileFlags cf)
{
  Compile_dst_op(cf, &CodeGenerator::add, &CodeGenerator::add, true, false);
}

void CPU::NewRec::X64Compiler::Compile_sub(CompileFlags cf)
{
  Compile_dst_op(cf, &CodeGenerator::sub, &CodeGenerator::sub, false, true);
}

void CPU::NewRec::X64Compiler::Compile_subu(CompileFlags cf)
{
  Compile_dst_op(cf, &CodeGenerator::sub, &CodeGenerator::sub, false, false);
}

void CPU::NewRec::X64Compiler::Compile_and(CompileFlags cf)
{
  // special cases - and with self -> self, and with 0 -> 0
  const Reg32 regd = CFGetRegD(cf);
  if (cf.MipsS() == cf.MipsT())
  {
    MoveSToReg(regd, cf);
    return;
  }
  else if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0))
  {
    cg->xor_(regd, regd);
    return;
  }

  Compile_dst_op(cf, &CodeGenerator::and_, &CodeGenerator::and_, true, false);
}

void CPU::NewRec::X64Compiler::Compile_or(CompileFlags cf)
{
  // or/nor with 0 -> no effect
  const Reg32 regd = CFGetRegD(cf);
  if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0) || cf.MipsS() == cf.MipsT())
  {
    cf.const_s ? MoveTToReg(regd, cf) : MoveSToReg(regd, cf);
    return;
  }

  Compile_dst_op(cf, &CodeGenerator::or_, &CodeGenerator::or_, true, false);
}

void CPU::NewRec::X64Compiler::Compile_xor(CompileFlags cf)
{
  const Reg32 regd = CFGetRegD(cf);
  if (cf.MipsS() == cf.MipsT())
  {
    // xor with self -> zero
    cg->xor_(regd, regd);
    return;
  }
  else if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0))
  {
    // xor with zero -> no effect
    cf.const_s ? MoveTToReg(regd, cf) : MoveSToReg(regd, cf);
    return;
  }

  Compile_dst_op(cf, &CodeGenerator::xor_, &CodeGenerator::xor_, true, false);
}

void CPU::NewRec::X64Compiler::Compile_nor(CompileFlags cf)
{
  Compile_or(cf);
  cg->not_(CFGetRegD(cf));
}

void CPU::NewRec::X64Compiler::Compile_slt(CompileFlags cf)
{
  Compile_slt(cf, true);
}

void CPU::NewRec::X64Compiler::Compile_sltu(CompileFlags cf)
{
  Compile_slt(cf, false);
}

void CPU::NewRec::X64Compiler::Compile_slt(CompileFlags cf, bool sign)
{
  const Reg32 rd = CFGetRegD(cf);
  const Reg32 rs = cf.valid_host_s ? CFGetRegS(cf) : RWARG1;
  const Reg32 rt = cf.valid_host_t ? CFGetRegT(cf) : RWARG1;
  if (!cf.valid_host_s)
    MoveSToReg(rs, cf);

  // Case where D == S, can't use xor because it changes flags
  // TODO: swap and reverse op for constants
  if (rd != rs && rd != rt)
    cg->xor_(rd, rd);

  if (cf.valid_host_t)
    cg->cmp(rs, CFGetRegT(cf));
  else if (cf.const_t)
    cg->cmp(rs, GetConstantRegU32(cf.MipsT()));
  else
    cg->cmp(rs, MipsPtr(cf.MipsT()));

  if (rd == rs || rd == rt)
    cg->mov(rd, 0);

  sign ? cg->setl(rd.cvt8()) : cg->setb(rd.cvt8());
}

void CPU::NewRec::X64Compiler::ComputeLoadStoreAddressArg(CompileFlags cf,
                                                          const std::optional<VirtualMemoryAddress>& address)
{
  if (address.has_value())
  {
    cg->mov(RWARG1, address.value());
  }
  else
  {
    if (cf.valid_host_s)
      cg->mov(RWARG1, CFGetRegS(cf));
    else
      cg->mov(RWARG1, MipsPtr(cf.MipsS()));

    if (const u32 imm = inst->i.imm_sext32(); imm != 0)
      cg->add(RWARG1, inst->i.imm_sext32());
  }
}

void CPU::NewRec::X64Compiler::Compile_Load(CompileFlags cf, MemoryAccessSize size, bool sign,
                                            const std::optional<VirtualMemoryAddress>& address)
{
  ComputeLoadStoreAddressArg(cf, address);
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);

  switch (size)
  {
    case MemoryAccessSize::Byte:
      cg->call(&Recompiler::Thunks::UncheckedReadMemoryByte);
      break;
    case MemoryAccessSize::HalfWord:
      cg->call(&Recompiler::Thunks::UncheckedReadMemoryHalfWord);
      break;
    case MemoryAccessSize::Word:
      cg->call(&Recompiler::Thunks::UncheckedReadMemoryWord);
      break;
  }

  if (cf.MipsT() == Reg::zero)
    return;

  // TODO: option to turn off load delay?
  Reg32 rt;
  if constexpr (true)
    rt = Reg32(AllocateHostReg(HR_MODE_WRITE, Compiler::HR_TYPE_NEXT_LOAD_DELAY_VALUE, cf.mips_t));
  else
    rt = Reg32(AllocateHostReg(HR_MODE_WRITE, Compiler::HR_TYPE_CPU_REG, cf.mips_t));

  switch (size)
  {
    case MemoryAccessSize::Byte:
      sign ? cg->movsx(rt, RWRET.cvt8()) : cg->movzx(rt, RWRET.cvt8());
      break;
    case MemoryAccessSize::HalfWord:
      sign ? cg->movsx(rt, RWRET.cvt16()) : cg->movzx(rt, RWRET.cvt16());
      break;
    case MemoryAccessSize::Word:
      cg->mov(rt, RWRET);
      break;
  }
}

void CPU::NewRec::X64Compiler::Compile_Store(CompileFlags cf, MemoryAccessSize size,
                                             const std::optional<VirtualMemoryAddress>& address)
{
  ComputeLoadStoreAddressArg(cf, address);
  MoveTToReg(RWARG2, cf);
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);

  switch (size)
  {
    case MemoryAccessSize::Byte:
      cg->call(&Recompiler::Thunks::UncheckedWriteMemoryByte);
      break;
    case MemoryAccessSize::HalfWord:
      cg->call(&Recompiler::Thunks::UncheckedWriteMemoryHalfWord);
      break;
    case MemoryAccessSize::Word:
      cg->call(&Recompiler::Thunks::UncheckedWriteMemoryWord);
      break;
  }
}

void CPU::NewRec::X64Compiler::CompileThunk(u32 start_pc)
{
  // TODO: share this

  Block* block = LookupBlock(start_pc);
  if (block)
  {
    // we should only be here if the block got invalidated
    DebugAssert(block->invalidated);
    if (RevalidateBlock(block))
    {
      SetFastMap(start_pc, block->host_code);
      return;
    }
  }

  block = CreateBlock(start_pc);
  if (!block)
    Panic("Failed to create block, TODO fallback to interpreter");

  // TODO: Persist the compiler
  CPU::NewRec::X64Compiler cc;
  block->host_code = cc.CompileBlock(block);
  if (!block->host_code)
  {
    // block failed to compile
    block->host_code = &CPU::CodeCache::InterpretUncachedBlock<PGXPMode::Disabled>;
  }

  SetFastMap(start_pc, block->host_code);
}

void CPU::NewRec::X64Compiler::CompileASMFunctions()
{
  CodeGenerator acg(g_code_buffer.GetFreeCodeSpace(), g_code_buffer.GetFreeCodePointer());
  CodeGenerator* cg = &acg;

  Label event_test_and_dispatch;
  Label dispatch;
  Label exit_recompiler;

  u32 stack_save_size = 0;
  u32 stack_offset = 0;
  g_enter_recompiler = cg->getCurr();
  {
    // Save all callee-saved regs so we don't need to.
    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      if (!IsCallerSavedRegister(i))
        stack_save_size += sizeof(void*);
    }
    stack_save_size += Common::IsAlignedPow2(stack_save_size + 8, 16) ? 0 : 8;
#ifdef _WIN32
    // Shadow space for Win32
    stack_save_size += 32;
    stack_offset = 32;
#endif
    cg->sub(cg->rsp, stack_save_size);
    for (u32 i = 0, cur_stack_offset = stack_offset; i < NUM_HOST_REGS; i++)
    {
      if (!IsCallerSavedRegister(i))
      {
        cg->mov(cg->qword[cg->rsp + cur_stack_offset], Reg64(i));
        cur_stack_offset += sizeof(void*);
      }
    }

    // Fall through to event dispatcher
  }

  // check events then for frame done
  g_event_test_and_dispatch = cg->getCurr();
  {
    cg->L(event_test_and_dispatch);

    Label check_interrupts;
    cg->mov(RXARG1, cg->qword[PTR(TimingEvents::GetHeadEventPtr())]);
    cg->mov(RWARG2, cg->dword[RXARG1 + offsetof(TimingEvent, m_downcount)]);
    cg->cmp(RWARG2, cg->dword[PTR(&g_state.pending_ticks)]);
    cg->jg(check_interrupts, CodeGenerator::T_SHORT);
    cg->call(reinterpret_cast<const void*>(&TimingEvents::RunEvents));

    cg->L(check_interrupts);

    // eax <- sr
    cg->mov(RWARG1, cg->dword[PTR(&g_state.cop0_regs.sr.bits)]);

    // if Iec == 0 then goto no_interrupt
    cg->test(RWARG1, 1);
    cg->jz(dispatch, Xbyak::CodeGenerator::T_SHORT);

    // sr & cause
    cg->and_(RWARG1, cg->dword[PTR(&g_state.cop0_regs.cause.bits)]);

    // ((sr & cause) & 0xff00) == 0 goto no_interrupt
    cg->test(RWARG1, 0xFF00);
    cg->jz(dispatch, Xbyak::CodeGenerator::T_SHORT);

    // we have an interrupt
    cg->call(reinterpret_cast<const void*>(&DispatchInterrupt));
  }

  // TODO: align?
  g_dispatcher = cg->getCurr();
  {
    cg->L(dispatch);

    // rcx <- s_fast_map[pc >> 16]
    cg->mov(RWARG1, cg->dword[PTR(&g_state.regs.pc)]);
    cg->lea(RXARG2, cg->dword[PTR(g_fast_map.data())]);
    cg->mov(RWARG3, RWARG1);
    cg->shr(RWARG3, 16);
    cg->mov(RXARG2, cg->qword[RXARG2 + RXARG3 * 8]);

    // call(rcx[pc * 2]) (fast_map[pc >> 2])
    cg->jmp(cg->qword[RXARG2 + RXARG1 * 2]);
  }

  g_compile_block = cg->getCurr();
  {
    cg->mov(RWARG1, cg->dword[PTR(&g_state.regs.pc)]);
    cg->call(&X64Compiler::CompileThunk);
    cg->jmp(dispatch);
  }

  g_exit_recompiler = cg->getCurr();
  {
    cg->L(exit_recompiler);

    // TODO: reverse order
    for (u32 i = 0, cur_stack_offset = stack_offset; i < NUM_HOST_REGS; i++)
    {
      if (!IsCallerSavedRegister(i))
      {
        cg->mov(Reg64(i), cg->qword[cg->rsp + cur_stack_offset]);
        cur_stack_offset += sizeof(void*);
      }
    }

    // TODO: Fixme
    // cg->ret(stack_save_size);
    cg->add(cg->rsp, stack_save_size);
    cg->ret();
  }

  g_code_buffer.CommitCode(static_cast<u32>(cg->getSize()));
}

void CPU::NewRec::CompileASMFunctions()
{
  X64Compiler::CompileASMFunctions();
}
