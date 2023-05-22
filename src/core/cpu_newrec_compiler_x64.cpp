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
#include "gte.h"
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

// TODO: register renaming, obviously
// TODO: try using a pointer to state instead of rip-relative.. it might end up faster due to smaller code

namespace CPU::CodeCache {
void LogCurrentState();
}

namespace CPU::NewRec {
X64Compiler s_instance;
Compiler* CPU::NewRec::g_compiler = &s_instance;
} // namespace CPU::NewRec

CPU::NewRec::X64Compiler::X64Compiler() = default;

CPU::NewRec::X64Compiler::~X64Compiler() = default;

bool CPU::NewRec::X64Compiler::IsCallerSavedRegister(u32 id)
{
#ifdef _WIN32
  // The x64 ABI considers the registers RAX, RCX, RDX, R8, R9, R10, R11, and XMM0-XMM5 volatile.
  return (id <= 2 || (id >= 8 && id <= 11));
#else
  // rax, rdi, rsi, rdx, rcx, r8, r9, r10, r11 are scratch registers.
  return (id <= 2 || id == 6 || id == 7 || (id >= 8 && id <= 11));
#endif
}

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
#if 0
  cg->call(&CPU::CodeCache::LogCurrentState);
#endif

#if 0
  if (m_block->pc == 0x000029C4)
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

void CPU::NewRec::X64Compiler::LoadHostRegWithConstant(u32 reg, u32 val)
{
  cg->mov(Reg32(reg), val);
}

void CPU::NewRec::X64Compiler::LoadHostRegFromCPUPointer(u32 reg, const void* ptr)
{
  cg->mov(Reg32(reg), cg->dword[PTR(ptr)]);
}

void CPU::NewRec::X64Compiler::StoreHostRegToCPUPointer(u32 reg, const void* ptr)
{
  cg->mov(cg->dword[PTR(ptr)], Reg32(reg));
}

void CPU::NewRec::X64Compiler::StoreConstantToCPUPointer(u32 val, const void* ptr)
{
  cg->mov(cg->dword[PTR(ptr)], val);
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
  MoveSToReg(rd, cf);

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

  if (flags & FLUSH_LOAD_DELAY_FROM_STATE && m_load_delay_dirty)
  {
    // This sucks :(
    // TODO: make it a function?
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

  if (flags & FLUSH_GTE_STALL_FROM_STATE && m_dirty_gte_done_cycle)
  {
    // May as well flush cycles while we're here.
    // GTE spanning blocks is very rare, we _could_ disable this for speed.
    cg->mov(RWARG1, cg->dword[PTR(&g_state.pending_ticks)]);
    cg->mov(RWARG2, cg->dword[PTR(&g_state.gte_completion_tick)]);
    if (m_cycles > 0)
    {
      (m_cycles == 1) ? cg->inc(RWARG1) : cg->add(RWARG1, m_cycles);
      m_cycles = 0;
    }
    cg->cmp(RWARG2, RWARG1);
    cg->cmova(RWARG1, RWARG2);
    cg->mov(cg->dword[PTR(&g_state.pending_ticks)], RWARG1);
    m_dirty_gte_done_cycle = false;
  }

  if (flags & FLUSH_GTE_DONE_CYCLE && m_gte_done_cycle > m_cycles)
  {
    cg->mov(RWARG1, cg->dword[PTR(&g_state.pending_ticks)]);

    // update cycles at the same time
    if (flags & FLUSH_CYCLES && m_cycles > 0)
    {
      (m_cycles == 1) ? cg->inc(RWARG1) : cg->add(RWARG1, m_cycles);
      cg->mov(cg->dword[PTR(&g_state.pending_ticks)], RWARG1);
      m_gte_done_cycle -= m_cycles;
      m_cycles = 0;
    }

    (m_gte_done_cycle == 1) ? cg->inc(RWARG1) : cg->add(RWARG1, m_gte_done_cycle);
    cg->mov(cg->dword[PTR(&g_state.gte_completion_tick)], RWARG1);
    m_gte_done_cycle = 0;
    m_dirty_gte_done_cycle = true;
  }

  if (flags & FLUSH_CYCLES && m_cycles > 0)
  {
    (m_cycles == 1) ? cg->inc(cg->dword[PTR(&g_state.pending_ticks)]) :
                      cg->add(cg->dword[PTR(&g_state.pending_ticks)], m_cycles);
    m_gte_done_cycle = std::max<TickCount>(m_gte_done_cycle - m_cycles, 0);
    m_cycles = 0;
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

  m_load_delay_dirty = EMULATE_LOAD_DELAYS;
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

  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetBranchReturnAddress());

  cg->mov(cg->dword[PTR(&g_state.regs.pc)], pcreg);
  CheckBranchTarget(pcreg);
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

void CPU::NewRec::X64Compiler::Compile_div(CompileFlags cf)
{
  // not supported without registers for now..
  DebugAssert(cf.valid_host_lo && cf.valid_host_hi);

  const Reg32 rt = cf.valid_host_t ? CFGetRegT(cf) : cg->ecx;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  const Reg32 rlo = CFGetRegLO(cf);
  const Reg32 rhi = CFGetRegHI(cf);

  MoveSToReg(cg->eax, cf);
  cg->cdq();

  Label done;
  Label not_divide_by_zero;
  cg->test(rt, rt);
  cg->jnz(not_divide_by_zero, CodeGenerator::T_SHORT);
  cg->test(cg->eax, cg->eax);
  cg->mov(rhi, cg->eax); // hi = num
  cg->mov(rlo, 1);
  cg->mov(cg->eax, static_cast<u32>(-1));
  cg->cmovns(rlo, cg->eax); // lo = s >= 0 ? -1 : 1
  cg->jmp(done, CodeGenerator::T_SHORT);

  cg->L(not_divide_by_zero);
  Label not_unrepresentable;
  cg->cmp(cg->eax, 0x80000000u);
  cg->jne(not_unrepresentable, CodeGenerator::T_SHORT);
  cg->cmp(rt, static_cast<u32>(-1));
  cg->jne(not_unrepresentable, CodeGenerator::T_SHORT);

  cg->mov(rlo, 0x80000000u);
  cg->xor_(rhi, rhi);
  cg->jmp(done, CodeGenerator::T_SHORT);

  cg->L(not_unrepresentable);

  cg->idiv(rt);
  cg->mov(rlo, cg->eax);
  cg->mov(rhi, cg->edx);

  cg->L(done);
}

void CPU::NewRec::X64Compiler::Compile_divu(CompileFlags cf)
{
  // not supported without registers for now..
  DebugAssert(cf.valid_host_lo && cf.valid_host_hi);

  const Reg32 rt = cf.valid_host_t ? CFGetRegT(cf) : cg->ecx;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  const Reg32 rlo = CFGetRegLO(cf);
  const Reg32 rhi = CFGetRegHI(cf);

  MoveSToReg(cg->eax, cf);
  cg->xor_(cg->edx, cg->edx);

  Label done;
  Label not_divide_by_zero;
  cg->test(rt, rt);
  cg->jnz(not_divide_by_zero, CodeGenerator::T_SHORT);
  cg->mov(rlo, static_cast<u32>(-1));
  cg->mov(rhi, cg->eax);
  cg->jmp(done, CodeGenerator::T_SHORT);

  cg->L(not_divide_by_zero);
  cg->div(rt);
  cg->mov(rlo, cg->eax);
  cg->mov(rhi, cg->edx);

  cg->L(done);
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

void CPU::NewRec::X64Compiler::ComputeLoadStoreAddressArg(Reg32 dst, CompileFlags cf,
                                                          const std::optional<VirtualMemoryAddress>& address)
{
  if (address.has_value())
  {
    cg->mov(dst, address.value());
  }
  else
  {
    if (cf.valid_host_s)
    {
      if (const Reg32 src = CFGetRegS(cf); src != dst)
        cg->mov(dst, CFGetRegS(cf));
    }
    else
    {
      cg->mov(dst, MipsPtr(cf.MipsS()));
    }

    if (const u32 imm = inst->i.imm_sext32(); imm != 0)
      cg->add(dst, inst->i.imm_sext32());
  }
}

void CPU::NewRec::X64Compiler::Compile_lxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                           const std::optional<VirtualMemoryAddress>& address)
{
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);
  ComputeLoadStoreAddressArg(RWARG1, cf, address);

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

  // TODO: move this out
  if (inst->op == InstructionOp::lwc2)
  {
    const u32 index = static_cast<u32>(inst->r.rt.GetValue());
    const auto [ptr, action] = GetGTERegisterPointer(index, true);
    switch (action)
    {
      case GTERegisterAccessAction::Ignore:
      {
        return;
      }

      case GTERegisterAccessAction::Direct:
      {
        cg->mov(cg->dword[PTR(ptr)], RWRET);
        return;
      }

      case GTERegisterAccessAction::SignExtend16:
      {
        cg->movsx(RWRET, RWRET.cvt16());
        cg->mov(cg->dword[PTR(ptr)], RWRET);
        return;
      }

      case GTERegisterAccessAction::ZeroExtend16:
      {
        cg->movzx(RWRET, RWRET.cvt16());
        cg->mov(cg->dword[PTR(ptr)], RWRET);
        return;
      }

      case GTERegisterAccessAction::CallHandler:
      {
        Flush(FLUSH_FOR_C_CALL);
        cg->mov(RWARG2, RWRET);
        cg->mov(RWARG1, index);
        cg->call(&GTE::WriteRegister);
        return;
      }

      case GTERegisterAccessAction::PushFIFO:
      {
        // SXY0 <- SXY1
        // SXY1 <- SXY2
        // SXY2 <- SXYP
        DebugAssert(RWRET != RWARG1 && RWRET != RWARG2);
        cg->mov(RWARG1, cg->dword[PTR(&g_state.gte_regs.SXY1[0])]);
        cg->mov(RWARG2, cg->dword[PTR(&g_state.gte_regs.SXY2[0])]);
        cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY0[0])], RWARG1);
        cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY1[0])], RWARG2);
        cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY2[0])], RWRET);
        return;
      }

      default:
      {
        Panic("Unknown action");
        return;
      }
    }
  }

  if (cf.MipsT() == Reg::zero)
    return;

  // TODO: option to turn off load delay?
  const Reg32 rt = Reg32(
    AllocateHostReg(HR_MODE_WRITE, EMULATE_LOAD_DELAYS ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG, cf.MipsT()));

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

void CPU::NewRec::X64Compiler::Compile_lwx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                           const std::optional<VirtualMemoryAddress>& address)
{
  DebugAssert(EMULATE_LOAD_DELAYS);

  // TODO: if address is constant, this can be simplified..
  // TODO: Share this
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);

  // We'd need to be careful here if we weren't overwriting it..
  const Reg32 addr = Reg32(AllocateHostReg(HR_CALLEE_SAVED, HR_TYPE_TEMP));
  ComputeLoadStoreAddressArg(addr, cf, address);
  cg->mov(RWARG1, addr);
  cg->and_(RWARG1, ~0x3u);

  cg->call(&Recompiler::Thunks::UncheckedReadMemoryWord);

  if (inst->r.rt == Reg::zero)
  {
    FreeHostReg(addr.getIdx());
    return;
  }

  // TODO: this can take over rt's value if it's no longer needed
  // NOTE: can't trust T in cf because of the flush
  const Reg rt = inst->r.rt;
  const Reg32 value = Reg32(AllocateHostReg(HR_MODE_WRITE, HR_TYPE_NEXT_LOAD_DELAY_VALUE, rt));
  DebugAssert(value != cg->ecx);
  if (HasConstantReg(rt))
    cg->mov(value, GetConstantRegU32(rt));
  else if (const std::optional<u32> rtreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt); rtreg.has_value())
    cg->mov(value, Reg32(rtreg.value()));
  else
    cg->mov(value, MipsPtr(rt));

  cg->mov(cg->ecx, addr);
  cg->and_(cg->ecx, 3);
  cg->shl(cg->ecx, 3); // *8

  // TODO for other arch: reverse subtract
  DebugAssert(RWARG2 != cg->ecx);
  cg->mov(RWARG2, 24);
  cg->sub(RWARG2, cg->ecx);

  if (inst->op == InstructionOp::lwl)
  {
    // const u32 mask = UINT32_C(0x00FFFFFF) >> shift;
    // new_value = (value & mask) | (RWRET << (24 - shift));
    cg->mov(addr, 0xFFFFFFu);
    cg->shr(addr, cg->cl);
    cg->and_(value, addr);
    cg->mov(cg->ecx, RWARG2);
    cg->shl(RWRET, cg->cl);
    cg->or_(value, RWRET);
  }
  else
  {
    // const u32 mask = UINT32_C(0xFFFFFF00) << (24 - shift);
    // new_value = (value & mask) | (RWRET >> shift);
    cg->shr(RWRET, cg->cl);
    cg->mov(addr, 0xFFFFFF00u);
    cg->mov(cg->ecx, RWARG2);
    cg->shl(addr, cg->cl);
    cg->and_(value, addr);
    cg->or_(value, RWRET);
  }

  FreeHostReg(addr.getIdx());
}

void CPU::NewRec::X64Compiler::Compile_sxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                           const std::optional<VirtualMemoryAddress>& address)
{
  // TODO: Stores don't need to flush GTE cycles...
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);

  ComputeLoadStoreAddressArg(RWARG1, cf, address);

  // TODO: move this out
  if (inst->op == InstructionOp::swc2)
  {
    const u32 index = static_cast<u32>(inst->r.rt.GetValue());
    const auto [ptr, action] = GetGTERegisterPointer(index, false);
    switch (action)
    {
      case GTERegisterAccessAction::Direct:
      {
        cg->mov(RWARG2, cg->dword[PTR(ptr)]);
      }
      break;

      case GTERegisterAccessAction::CallHandler:
      {
        Flush(FLUSH_FOR_C_CALL);
        cg->mov(RWARG1, index);
        cg->call(&GTE::ReadRegister);
        cg->mov(RWRET, RWARG2);
      }
      break;

      default:
      {
        Panic("Unknown action");
      }
      break;
    }
  }
  else
  {
    MoveTToReg(RWARG2, cf);
  }

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

void CPU::NewRec::X64Compiler::Compile_swx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                           const std::optional<VirtualMemoryAddress>& address)
{
  DebugAssert(EMULATE_LOAD_DELAYS);

  // TODO: if address is constant, this can be simplified..
  // TODO: Share this
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);

  // We'd need to be careful here if we weren't overwriting it..
  const Reg32 addr = Reg32(AllocateHostReg(HR_CALLEE_SAVED, HR_TYPE_TEMP));
  ComputeLoadStoreAddressArg(addr, cf, address);
  cg->mov(RWARG1, addr);
  cg->and_(RWARG1, ~0x3u);

  cg->call(&Recompiler::Thunks::UncheckedReadMemoryWord);

  // TODO: this can take over rt's value if it's no longer needed
  // NOTE: can't trust T in cf because of the flush
  const Reg rt = inst->r.rt;
  const Reg32 value = RWARG2;
  DebugAssert(value != cg->ecx);
  if (HasConstantReg(rt))
    cg->mov(value, GetConstantRegU32(rt));
  else if (const std::optional<u32> rtreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt); rtreg.has_value())
    cg->mov(value, Reg32(rtreg.value()));
  else
    cg->mov(value, MipsPtr(rt));

  cg->mov(cg->ecx, addr);
  cg->and_(cg->ecx, 3);
  cg->shl(cg->ecx, 3); // *8

  // TODO for other arch: reverse subtract
  DebugAssert(RWARG3 != cg->ecx);
  cg->mov(RWARG3, 24);
  cg->sub(RWARG3, cg->ecx);

  if (inst->op == InstructionOp::swl)
  {
    // const u32 mem_mask = UINT32_C(0xFFFFFF00) << shift;
    // new_value = (RWRET & mem_mask) | (value >> (24 - shift));
    cg->mov(RWARG3, 0xFFFFFF00u);
    cg->shl(RWARG3, cg->cl);
    cg->and_(RWRET, RWARG3);

    cg->mov(RWARG3, 24);
    cg->sub(RWARG3, cg->ecx);
    cg->mov(cg->ecx, RWARG3);
    cg->shr(value, cg->cl);
    cg->or_(value, RWRET);
  }
  else
  {
    // const u32 mem_mask = UINT32_C(0x00FFFFFF) >> (24 - shift);
    // new_value = (RWRET & mem_mask) | (value << shift);
    cg->shl(value, cg->cl);

    cg->mov(RWARG3, 24);
    cg->sub(RWARG3, cg->ecx);
    cg->mov(cg->ecx, RWARG3);
    cg->mov(RWARG3, 0x00FFFFFFu);
    cg->shr(RWARG3, cg->cl);
    cg->and_(RWRET, RWARG3);
    cg->or_(value, RWRET);
  }

  FreeHostReg(addr.getIdx());

  cg->mov(RWARG1, addr);
  cg->and_(RWARG1, ~0x3u);
  cg->call(&Recompiler::Thunks::UncheckedWriteMemoryWord);
}

void CPU::NewRec::X64Compiler::Compile_mtc0(CompileFlags cf)
{
  const Cop0Reg reg = static_cast<Cop0Reg>(MipsD());
  const u32* ptr = GetCop0RegPtr(reg);
  const u32 mask = GetCop0RegWriteMask(reg);
  if (!ptr)
  {
    Compile_Fallback();
    return;
  }

  const Reg32 rt = cf.valid_host_t ? CFGetRegT(cf) : RWARG1;
  const u32 constant_value = cf.const_t ? GetConstantRegU32(cf.MipsT()) : 0;
  if (mask == 0)
  {
    // if it's a read-only register, ignore
    Log_DebugPrintf("Ignoring write to read-only cop0 reg %u", static_cast<u32>(reg));
    return;
  }

  // for some registers, we need to test certain bits
  const bool needs_bit_test = (reg == Cop0Reg::SR && g_settings.IsUsingFastmem());
  const Reg32 changed_bits = RWARG3;

  // update value
  if (cf.valid_host_t)
  {
    cg->mov(RWARG1, rt);
    cg->mov(RWARG2, cg->dword[PTR(ptr)]);
    cg->and_(RWARG1, mask);
    if (needs_bit_test)
    {
      cg->mov(changed_bits, RWARG2);
      cg->xor_(changed_bits, rt);
    }
    cg->and_(RWARG2, ~mask);
    cg->or_(RWARG2, rt);
    cg->mov(cg->dword[PTR(ptr)], RWARG2);
  }
  else
  {
    cg->mov(RWARG2, cg->dword[PTR(ptr)]);
    if (needs_bit_test)
    {
      cg->mov(changed_bits, RWARG2);
      cg->xor_(changed_bits, constant_value);
    }
    cg->and_(RWARG2, ~mask);
    cg->or_(RWARG2, constant_value & mask);
    cg->mov(cg->dword[PTR(ptr)], RWARG2);
  }

  if (reg == Cop0Reg::SR && g_settings.IsUsingFastmem())
  {
    // TODO: changing SR[Isc] needs to update fastmem views
    Log_WarningPrintf("TODO: changing SR[Isc] needs to update fastmem views");
  }

  if (reg == Cop0Reg::SR || reg == Cop0Reg::CAUSE)
  {
    const Reg32 sr =
      (reg == Cop0Reg::SR) ? RWARG2 : (cg->mov(RWARG1, cg->dword[PTR(&g_state.cop0_regs.sr.bits)]), RWARG1);
    TestInterrupts(sr);
  }

  if (reg == Cop0Reg::DCIC && g_settings.cpu_recompiler_memory_exceptions)
  {
    // TODO: DCIC handling for debug breakpoints
    Log_WarningPrintf("TODO: DCIC handling for debug breakpoints");
  }
}

void CPU::NewRec::X64Compiler::Compile_rfe(CompileFlags cf)
{
  // shift mode bits right two, preserving upper bits
  static constexpr u32 mode_bits_mask = UINT32_C(0b1111);
  cg->mov(RWARG1, cg->dword[PTR(&g_state.cop0_regs.sr.bits)]);
  cg->mov(RWARG2, RWARG1);
  cg->shr(RWARG2, 2);
  cg->and_(RWARG1, ~mode_bits_mask);
  cg->and_(RWARG2, mode_bits_mask);
  cg->or_(RWARG1, RWARG2);
  cg->mov(cg->dword[PTR(&g_state.cop0_regs.sr.bits)], RWARG1);

  TestInterrupts(RWARG1);
}

void CPU::NewRec::X64Compiler::TestInterrupts(const Xbyak::Reg32& sr)
{
  // need to test for interrupts, flush everything back, since we're gonna have to do it anyway, and this isn't hot
  Flush(FLUSH_FLUSH_MIPS_REGISTERS | FLUSH_FOR_INTERRUPT);

  // if Iec == 0 then goto no_interrupt
  Label no_interrupt;

  cg->test(sr, 1);
  cg->jz(no_interrupt, Xbyak::CodeGenerator::T_SHORT);

  // sr & cause
  cg->and_(sr, cg->dword[PTR(&g_state.cop0_regs.cause.bits)]);

  // ((sr & cause) & 0xff00) == 0 goto no_interrupt
  cg->test(sr, 0xFF00);
  cg->jz(no_interrupt, Xbyak::CodeGenerator::T_SHORT);

  // TODO: put this in far code
  BackupHostState();
  Flush(FLUSH_FOR_C_CALL);
  cg->call(reinterpret_cast<const void*>(&DispatchInterrupt));
  EndBlock(std::nullopt);
  RestoreHostState();

  cg->L(no_interrupt);
}

void CPU::NewRec::X64Compiler::Compile_mfc2(CompileFlags cf)
{
  const u32 index = inst->cop.Cop2Index();
  const Reg rt = inst->r.rt;

  const auto [ptr, action] = GetGTERegisterPointer(index, false);
  if (action == GTERegisterAccessAction::Ignore)
    return;

  if (action == GTERegisterAccessAction::Direct)
  {
    const u32 hreg =
      AllocateHostReg(HR_MODE_WRITE, EMULATE_LOAD_DELAYS ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG, rt);
    cg->mov(Reg32(hreg), cg->dword[PTR(ptr)]);
  }
  else if (action == GTERegisterAccessAction::CallHandler)
  {
    Flush(FLUSH_FOR_C_CALL);
    cg->mov(RWARG1, index);
    cg->call(&GTE::ReadRegister);

    const u32 hreg =
      AllocateHostReg(HR_MODE_WRITE, EMULATE_LOAD_DELAYS ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG, rt);
    cg->mov(Reg32(hreg), RWRET);
  }
  else
  {
    Panic("Unknown action");
  }
}

void CPU::NewRec::X64Compiler::Compile_mtc2(CompileFlags cf)
{
  const u32 index = inst->cop.Cop2Index();
  const auto [ptr, action] = GetGTERegisterPointer(index, true);
  if (action == GTERegisterAccessAction::Ignore)
    return;

  if (action == GTERegisterAccessAction::Direct)
  {
    if (cf.const_t)
    {
      cg->mov(cg->dword[PTR(ptr)], GetConstantRegU32(cf.MipsT()));
    }
    else if (cf.valid_host_t)
    {
      cg->mov(cg->dword[PTR(ptr)], CFGetRegT(cf));
    }
    else
    {
      cg->mov(RWARG1, MipsPtr(cf.MipsT()));
      cg->mov(cg->dword[PTR(ptr)], RWARG1);
    }
  }
  else if (action == GTERegisterAccessAction::SignExtend16 || action == GTERegisterAccessAction::ZeroExtend16)
  {
    const bool sign = (action == GTERegisterAccessAction::SignExtend16);
    if (cf.const_t)
    {
      const u16 cv = Truncate16(GetConstantRegU32(cf.MipsT()));
      cg->mov(cg->dword[PTR(ptr)], sign ? ::SignExtend16(cv) : ::ZeroExtend16(cv));
    }
    else if (cf.valid_host_t)
    {
      sign ? cg->movsx(RWARG1, Reg16(cf.host_t)) : cg->movzx(RWARG1, Reg16(cf.host_t));
      cg->mov(cg->dword[PTR(ptr)], RWARG1);
    }
    else
    {
      sign ? cg->movsx(RWARG1, cg->word[PTR(&g_state.regs.r[cf.mips_t])]) :
             cg->movzx(RWARG1, cg->word[PTR(&g_state.regs.r[cf.mips_t])]);
      cg->mov(cg->dword[PTR(ptr)], RWARG1);
    }
  }
  else if (action == GTERegisterAccessAction::CallHandler)
  {
    Flush(FLUSH_FOR_C_CALL);
    cg->mov(RWARG1, index);
    MoveTToReg(RWARG2, cf);
    cg->call(&GTE::WriteRegister);
  }
  else if (action == GTERegisterAccessAction::PushFIFO)
  {
    // SXY0 <- SXY1
    // SXY1 <- SXY2
    // SXY2 <- SXYP
    cg->mov(RWARG1, cg->dword[PTR(&g_state.gte_regs.SXY1[0])]);
    cg->mov(RWARG2, cg->dword[PTR(&g_state.gte_regs.SXY2[0])]);
    if (!cf.const_t && !cf.valid_host_t)
      cg->mov(RWARG3, MipsPtr(cf.MipsT()));
    cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY0[0])], RWARG1);
    cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY1[0])], RWARG2);
    if (cf.const_t)
      cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY2[0])], GetConstantRegU32(cf.MipsT()));
    else if (cf.valid_host_t)
      cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY2[0])], CFGetRegT(cf));
    else
      cg->mov(cg->dword[PTR(&g_state.gte_regs.SXY2[0])], RWARG3);
  }
  else
  {
    Panic("Unknown action");
  }
}

void CPU::NewRec::X64Compiler::Compile_cop2(CompileFlags cf)
{
  TickCount func_ticks;
  GTE::InstructionImpl func = GTE::GetInstructionImpl(inst->bits, &func_ticks);

  Flush(FLUSH_FOR_C_CALL);
  cg->mov(RWARG1, inst->bits & GTE::Instruction::REQUIRED_BITS_MASK);
  cg->call(reinterpret_cast<const void*>(func));

  AddGTETicks(func_ticks);
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

    // Downcount isn't set on entry, so we need to initialize it
    cg->mov(RXARG1, cg->qword[PTR(TimingEvents::GetHeadEventPtr())]);
    cg->mov(RWARG1, cg->dword[RXARG1 + offsetof(TimingEvent, m_downcount)]);
    cg->mov(cg->dword[PTR(&g_state.downcount)], RWARG1);

    // Fall through to event dispatcher
  }

  // check events then for frame done
  g_event_test_and_dispatch = cg->getCurr();
  {
    cg->L(event_test_and_dispatch);

    Label check_interrupts;
    cg->mov(RXARG1, cg->qword[PTR(TimingEvents::GetHeadEventPtr())]);
    cg->mov(RWARG1, cg->dword[RXARG1 + offsetof(TimingEvent, m_downcount)]);
    cg->cmp(RWARG1, cg->dword[PTR(&g_state.pending_ticks)]);
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
    cg->call(&CompileOrRevalidateBlock);
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
