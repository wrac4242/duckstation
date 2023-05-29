// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "cpu_newrec_compiler_riscv64.h"
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
#include <limits>
Log_SetChannel(CPU::NewRec);

#define DUMP_BLOCKS

#ifdef DUMP_BLOCKS
extern "C" {
#include "riscv-disas.h"
}
#endif

using namespace biscuit;

#define RRET biscuit::a0
#define RARG1 biscuit::a0
#define RARG2 biscuit::a1
#define RARG3 biscuit::a2
#define RSCRATCH biscuit::t6
#define RSTATE biscuit::s10
#define RMEMBASE biscuit::s11

// For LW/SW/etc.
#define PTR(x) ((u32)(((u8*)(x)) - ((u8*)&g_state))), RSTATE

static constexpr u32 TRAMPOLINE_AREA_SIZE = 4 * 1024;

namespace CPU::CodeCache {
void LogCurrentState();
}

namespace CPU::NewRec {

// TODO: split into utils or something
static constexpr bool rvIsCallerSavedRegister(u32 id)
{
  return (id == 1 || (id >= 3 && id < 8) || (id >= 10 && id <= 17) || (id >= 28 && id <= 31));
}

static void rvDisassembleAndDumpCode(const void* ptr, size_t size);
static void rvFlushInstructionCache(void* start, u32 size);
#if 0
static s64 rvGetPCDisplacement(const void* current, const void* target);
#endif
static void rvCheck32BitDisplacement(const void* cur, const void* target);
static void rvMoveAddressToReg(Assembler* armAsm, const GPR& reg, const void* addr);
static void rvEmitMov(Assembler* rvAsm, const GPR& rd, u32 imm);
static void rvEmitJmp(Assembler* armAsm, const void* ptr, const GPR& link_reg = zero);
static void rvEmitCall(Assembler* armAsm, const void* ptr);
#if 0
static void armEmitCondBranch(Assembler* armAsm, vixl::aarch64::Condition cond, const void* ptr);
static u8* armGetJumpTrampoline(const void* target);
#endif

RISCV64Compiler s_instance;
Compiler* g_compiler = &s_instance;

static std::unordered_map<const void*, u32> s_trampoline_targets;
static u8* s_trampoline_start_ptr;
static u32 s_trampoline_used;
} // namespace CPU::NewRec

void CPU::NewRec::rvDisassembleAndDumpCode(const void* ptr, size_t size)
{
#ifdef DUMP_BLOCKS
  // TODO: FIXME
#else
  Log_DebugPrintf("Not compiled with DUMP_BLOCKS");
#endif
}

void CPU::NewRec::rvFlushInstructionCache(void* start, u32 size)
{
#if defined(__GNUC__) || defined(__clang__)
  __builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start) + size);
#else
  Panic("rvFlushInstructionCache() not implemented");
#endif
}

#if 0
s64 CPU::NewRec::armGetPCDisplacement(const void* current, const void* target)
{
  // pxAssert(Common::IsAlignedPow2(reinterpret_cast<size_t>(current), 4));
  // pxAssert(Common::IsAlignedPow2(reinterpret_cast<size_t>(target), 4));
  return static_cast<s64>((reinterpret_cast<ptrdiff_t>(target) - reinterpret_cast<ptrdiff_t>(current)) >> 2);
}
#endif

void CPU::NewRec::rvCheck32BitDisplacement(const void* cur, const void* target)
{
  // Doesn't support >32-bit for now.
  static constexpr uintptr_t mask = UINT64_C(0xFFFFFFFF);
  const uintptr_t curp = reinterpret_cast<uintptr_t>(cur);
  const uintptr_t addrp = reinterpret_cast<uintptr_t>(target);
  Assert((curp & ~mask) == (addrp & ~mask));
}

void CPU::NewRec::rvMoveAddressToReg(Assembler* rvAsm, const GPR& reg, const void* addr)
{
  rvCheck32BitDisplacement(rvAsm->GetCursorPointer(), addr);
  rvAsm->AUIPC(reg, static_cast<s32>(reinterpret_cast<u64>(addr) >> 12));
  rvAsm->ADDI(reg, reg, static_cast<s32>(reinterpret_cast<u64>(addr) & UINT64_C(0xFFF)));
}

void CPU::NewRec::rvEmitMov(Assembler* rvAsm, const GPR& rd, u32 imm)
{
  if (imm == 0)
  {
    rvAsm->ADD(rd, zero, zero);
    return;
  }

  rvAsm->LI(rd, imm);
}

void CPU::NewRec::rvEmitJmp(Assembler* rvAsm, const void* ptr, const GPR& link_reg)
{
  // TODO: use J if displacement is <1MB
  rvCheck32BitDisplacement(rvAsm->GetCursorPointer(), ptr);
  rvAsm->AUIPC(RSCRATCH, static_cast<s32>(reinterpret_cast<u64>(ptr) >> 12));
  rvAsm->JALR(link_reg, static_cast<s32>(reinterpret_cast<u64>(ptr) & UINT64_C(0xFFF)), RSCRATCH);
}

void CPU::NewRec::rvEmitCall(Assembler* rvAsm, const void* ptr)
{
  rvEmitJmp(rvAsm, ptr, ra);
}

#if 0
void CPU::NewRec::armEmitCondBranch(Assembler* armAsm, Condition cond, const void* ptr)
{
  const s64 jump_distance = static_cast<s64>(reinterpret_cast<intptr_t>(ptr) -
                                             reinterpret_cast<intptr_t>(armAsm->GetCursorAddress<const void*>()));
  // pxAssert(Common::IsAligned(jump_distance, 4));

  if (vixl::aarch64::Instruction::IsValidImmPCOffset(CondBranchType, jump_distance >> 2))
  {
    armAsm->b(jump_distance >> 2, cond);
  }
  else
  {
    Label branch_not_taken;
    armAsm->b(&branch_not_taken, InvertCondition(cond));

    const s64 new_jump_distance = static_cast<s64>(reinterpret_cast<intptr_t>(ptr) -
                                                   reinterpret_cast<intptr_t>(armAsm->GetCursorAddress<const void*>()));
    armAsm->b(new_jump_distance >> 2);
    armAsm->bind(&branch_not_taken);
  }
}

u8* CPU::NewRec::armGetJumpTrampoline(const void* target)
{
  auto it = s_trampoline_targets.find(target);
  if (it != s_trampoline_targets.end())
    return s_trampoline_start_ptr + it->second;

  // align to 16 bytes?
  const u32 offset = s_trampoline_used; // Common::AlignUpPow2(s_trampoline_used, 16);

  // 4 movs plus a jump
  if (TRAMPOLINE_AREA_SIZE - offset < 20)
  {
    Panic("Ran out of space in constant pool");
    return nullptr;
  }

  u8* start = s_trampoline_start_ptr + offset;
  Assembler armAsm(start, TRAMPOLINE_AREA_SIZE - offset);
  armMoveAddressToReg(&armAsm, RXSCRATCH, target);
  armAsm.br(RXSCRATCH);

  const u32 size = static_cast<u32>(armAsm.GetSizeOfCodeGenerated());
  DebugAssert(size < 20);
  s_trampoline_targets.emplace(target, offset);
  s_trampoline_used = offset + static_cast<u32>(size);

  rvFlushInstructionCache(start, size);
  return start;
}

#endif

CPU::NewRec::RISCV64Compiler::RISCV64Compiler() = default;

CPU::NewRec::RISCV64Compiler::~RISCV64Compiler() = default;

const void* CPU::NewRec::RISCV64Compiler::GetCurrentCodePointer()
{
  return rvAsm->GetCursorPointer();
}

void CPU::NewRec::RISCV64Compiler::Reset(Block* block, u8* code_buffer, u32 code_buffer_space, u8* far_code_buffer,
                                         u32 far_code_space)
{
  Compiler::Reset(block, code_buffer, code_buffer_space, far_code_buffer, far_code_space);

  // TODO: don't recreate this every time..
  DebugAssert(!m_emitter && !m_far_emitter && !rvAsm);
  m_emitter = std::make_unique<Assembler>(code_buffer, code_buffer_space);
  m_far_emitter = std::make_unique<Assembler>(far_code_buffer, far_code_space);
  rvAsm = m_emitter.get();

  // Need to wipe it out so it's correct when toggling fastmem.
  m_host_regs = {};

  const u32 membase_idx = g_settings.IsUsingFastmem() ? RMEMBASE.Index() : NUM_HOST_REGS;
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& hra = m_host_regs[i];

    if (i == RARG1.Index() || i == RARG2.Index() || i == RARG3.Index() || i == RSCRATCH.Index() ||
        i == RSTATE.Index() || i == membase_idx || i < 5 /* zero, ra, sp, gp, tp */)
    {
      continue;
    }

    hra.flags = HR_USABLE | (rvIsCallerSavedRegister(i) ? 0 : HR_CALLEE_SAVED);
  }
}

#if 0
void CPU::NewRec::RISCV64Compiler::SwitchToFarCode(bool emit_jump, vixl::aarch64::Condition cond)
{
  DebugAssert(rvAsm == m_emitter.get());
  if (emit_jump)
  {
    const s64 disp = armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>());
    if (cond != Condition::al)
    {
      if (vixl::IsInt19(disp))
      {
        rvAsm->b(disp, cond);
      }
      else
      {
        Label skip;
        rvAsm->b(&skip, vixl::aarch64::InvertCondition(cond));
        rvAsm->b(armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>()));
        rvAsm->bind(&skip);
      }
    }
    else
    {
      rvAsm->b(disp);
    }
  }
  rvAsm = m_far_emitter.get();
}

void CPU::NewRec::RISCV64Compiler::SwitchToFarCodeIfBitSet(const vixl::aarch64::Register& reg, u32 bit)
{
  const s64 disp = armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>());
  if (vixl::IsInt14(disp))
  {
    rvAsm->tbnz(reg, bit, disp);
  }
  else
  {
    Label skip;
    rvAsm->tbz(reg, bit, &skip);
    rvAsm->b(armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>()));
    rvAsm->bind(&skip);
  }

  rvAsm = m_far_emitter.get();
}

void CPU::NewRec::RISCV64Compiler::SwitchToFarCodeIfRegZeroOrNonZero(const vixl::aarch64::Register& reg, bool nonzero)
{
  const s64 disp = armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>());
  if (vixl::IsInt19(disp))
  {
    nonzero ? rvAsm->cbnz(reg, disp) : rvAsm->cbz(reg, disp);
  }
  else
  {
    Label skip;
    nonzero ? rvAsm->cbz(reg, &skip) : rvAsm->cbnz(reg, &skip);
    rvAsm->b(armGetPCDisplacement(GetCurrentCodePointer(), m_far_emitter->GetCursorAddress<const void*>()));
    rvAsm->bind(&skip);
  }

  rvAsm = m_far_emitter.get();
}

void CPU::NewRec::RISCV64Compiler::SwitchToNearCode(bool emit_jump, vixl::aarch64::Condition cond)
{
  DebugAssert(rvAsm == m_far_emitter.get());
  if (emit_jump)
  {
    const s64 disp = armGetPCDisplacement(GetCurrentCodePointer(), m_emitter->GetCursorAddress<const void*>());
    (cond != Condition::al) ? rvAsm->b(disp, cond) : rvAsm->b(disp);
  }
  rvAsm = m_emitter.get();
}

#endif

void CPU::NewRec::RISCV64Compiler::EmitMov(const biscuit::GPR& dst, u32 val)
{
  rvEmitMov(rvAsm, dst, val);
}

void CPU::NewRec::RISCV64Compiler::EmitCall(const void* ptr)
{
  rvEmitCall(rvAsm, ptr);
}

void CPU::NewRec::RISCV64Compiler::SafeImmSExtIType(const biscuit::GPR& rd, const biscuit::GPR& rs, u32 imm,
                                                    void (biscuit::Assembler::*iop)(GPR, GPR, u32),
                                                    void (biscuit::Assembler::*rop)(GPR, GPR, GPR))
{
  DebugAssert(rd != RSCRATCH && rs != RSCRATCH);

  if (static_cast<u32>((static_cast<s32>(imm) << 20) >> 20) == imm)
  {
    (rvAsm->*iop)(rd, rs, imm);
    return;
  }

  rvEmitMov(rvAsm, RSCRATCH, imm);
  (rvAsm->*rop)(rd, rs, RSCRATCH);
}

void CPU::NewRec::RISCV64Compiler::SafeANDI(const biscuit::GPR& rd, const biscuit::GPR& rs, u32 imm)
{
  SafeImmSExtIType(rd, rs, imm, &Assembler::ANDI, &Assembler::AND);
}

void CPU::NewRec::RISCV64Compiler::SafeORI(const biscuit::GPR& rd, const biscuit::GPR& rs, u32 imm)
{
  SafeImmSExtIType(rd, rs, imm, &Assembler::ORI, &Assembler::OR);
}

void CPU::NewRec::RISCV64Compiler::SafeXORI(const biscuit::GPR& rd, const biscuit::GPR& rs, u32 imm)
{
  SafeImmSExtIType(rd, rs, imm, &Assembler::XORI, &Assembler::XOR);
}

#if 0

vixl::aarch64::Operand CPU::NewRec::RISCV64Compiler::armCheckAddSubConstant(s32 val)
{
  if (Assembler::IsImmAddSub(val))
    return vixl::aarch64::Operand(static_cast<int64_t>(val));

  EmitMov(RWSCRATCH, static_cast<u32>(val));
  return vixl::aarch64::Operand(RWSCRATCH);
}

vixl::aarch64::Operand CPU::NewRec::RISCV64Compiler::armCheckAddSubConstant(u32 val)
{
  return armCheckAddSubConstant(static_cast<s32>(val));
}

vixl::aarch64::Operand CPU::NewRec::RISCV64Compiler::armCheckCompareConstant(s32 val)
{
  if (Assembler::IsImmConditionalCompare(val))
    return vixl::aarch64::Operand(static_cast<int64_t>(val));

  EmitMov(RWSCRATCH, static_cast<u32>(val));
  return vixl::aarch64::Operand(RWSCRATCH);
}

vixl::aarch64::Operand CPU::NewRec::RISCV64Compiler::armCheckLogicalConstant(u32 val)
{
  if (Assembler::IsImmLogical(val, 32))
    return vixl::aarch64::Operand(static_cast<s64>(static_cast<u64>(val)));

  EmitMov(RWSCRATCH, val);
  return vixl::aarch64::Operand(RWSCRATCH);
}

void CPU::NewRec::RISCV64Compiler::BeginBlock()
{
  Compiler::BeginBlock();
#if 0
  EmitCall(reinterpret_cast<const void*>(&CPU::CodeCache::LogCurrentState));
#endif
}

#endif

void CPU::NewRec::RISCV64Compiler::EndBlock(const std::optional<u32>& newpc)
{
  if (newpc.has_value())
  {
    if (m_dirty_pc || m_compiler_pc != newpc)
    {
      EmitMov(RSCRATCH, newpc.value());
      rvAsm->SW(RSCRATCH, PTR(&g_state.pc));
    }
  }
  m_dirty_pc = false;

  // flush regs
  Flush(FLUSH_END_BLOCK);
  EndAndLinkBlock(newpc);
}

void CPU::NewRec::RISCV64Compiler::EndBlockWithException(Exception excode)
{
  // flush regs, but not pc, it's going to get overwritten
  // flush cycles because of the GTE instruction stuff...
  Flush(FLUSH_END_BLOCK | FLUSH_FOR_EXCEPTION);

  // TODO: flush load delay
  // TODO: break for pcdrv

  EmitMov(RARG1, Cop0Registers::CAUSE::MakeValueForException(excode, m_current_instruction_branch_delay_slot, false,
                                                             inst->cop.cop_n));
  EmitMov(RARG2, m_current_instruction_pc);
  EmitCall(reinterpret_cast<const void*>(static_cast<void (*)(u32, u32)>(&CPU::RaiseException)));
  m_dirty_pc = false;

  EndAndLinkBlock(std::nullopt);
}

void CPU::NewRec::RISCV64Compiler::EndAndLinkBlock(const std::optional<u32>& newpc)
{
  // event test
  // pc should've been flushed
  DebugAssert(!m_dirty_pc);

  // TODO: try extracting this to a function
  // TODO: move the cycle flush in here..

  // save cycles for event test
  const TickCount cycles = std::exchange(m_cycles, 0);

  // pending_ticks += cycles
  // if (pending_ticks >= downcount) { dispatch_event(); }
  rvAsm->LW(RARG1, PTR(&g_state.pending_ticks));
  rvAsm->LW(RARG2, PTR(&g_state.downcount));
  if (cycles > 0)
  {
    SafeADDI(RARG1, RARG1, cycles);
    rvAsm->SW(RARG1, PTR(&g_state.pending_ticks));
  }
  Label cont;
  rvAsm->BLT(RARG1, RARG2, &cont);
  rvEmitJmp(rvAsm, g_check_events_and_dispatch);

  // jump to dispatcher or next block
  if (!newpc.has_value())
  {
    rvEmitJmp(rvAsm, g_dispatcher);
  }
  else
  {
    if (newpc.value() == m_block->pc)
    {
      // Special case: ourselves! No need to backlink then.
      Log_DebugPrintf("Linking block at %08X to self", m_block->pc);
      rvEmitJmp(rvAsm, rvAsm->GetBufferPointer(0));
    }
    else
    {
      void* placeholder = rvAsm->GetCursorPointer();
      rvAsm->NOP();
      rvAsm->NOP();
      const u32 size = CreateBlockLink(m_block, placeholder, newpc.value());
      DebugAssert(size == 8);
      UNREFERENCED_VARIABLE(size);
    }
  }

  m_block_ended = true;
}

const void* CPU::NewRec::RISCV64Compiler::EndCompile(u32* code_size, u32* far_code_size)
{
  u8* code = m_emitter->GetBufferPointer(0);
  const u32 my_code_size = static_cast<u32>(m_emitter->GetCodeBuffer().GetSizeInBytes());
  const u32 my_far_code_size = static_cast<u32>(m_emitter->GetCodeBuffer().GetSizeInBytes());

  if (my_code_size > 0)
    rvFlushInstructionCache(code, my_code_size);
  if (my_far_code_size > 0)
    rvFlushInstructionCache(m_far_emitter->GetCursorPointer(), my_far_code_size);

  *code_size = my_code_size;
  *far_code_size = my_far_code_size;
  rvAsm = nullptr;
  m_far_emitter.reset();
  m_emitter.reset();
  return code;
}

#if 0

void CPU::NewRec::RISCV64Compiler::DisassembleAndLog(const void* start, u32 size)
{
  rvDisassembleAndDumpCode(start, size);
}

u32 CPU::NewRec::RISCV64Compiler::GetHostInstructionCount(const void* start, u32 size)
{
  return size / kInstructionSize;
}

#endif

const char* CPU::NewRec::RISCV64Compiler::GetHostRegName(u32 reg) const
{
  static constexpr std::array<const char*, 32> reg64_names = {
    {"zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "s0", "s1", "a0",  "a1",  "a2", "a3", "a4", "a5",
     "a6",   "a7", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"}};
  return (reg < reg64_names.size()) ? reg64_names[reg] : "UNKNOWN";
}

void CPU::NewRec::RISCV64Compiler::LoadHostRegWithConstant(u32 reg, u32 val)
{
  EmitMov(GPR(reg), val);
}

void CPU::NewRec::RISCV64Compiler::LoadHostRegFromCPUPointer(u32 reg, const void* ptr)
{
  rvAsm->LW(GPR(reg), PTR(ptr));
}

void CPU::NewRec::RISCV64Compiler::StoreHostRegToCPUPointer(u32 reg, const void* ptr)
{
  rvAsm->SW(GPR(reg), PTR(ptr));
}

void CPU::NewRec::RISCV64Compiler::StoreConstantToCPUPointer(u32 val, const void* ptr)
{
  if (val == 0)
  {
    rvAsm->SW(zero, PTR(ptr));
    return;
  }

  EmitMov(RSCRATCH, val);
  rvAsm->SW(RSCRATCH, PTR(ptr));
}

void CPU::NewRec::RISCV64Compiler::CopyHostReg(u32 dst, u32 src)
{
  if (src != dst)
    rvAsm->MV(GPR(dst), GPR(src));
}

void CPU::NewRec::RISCV64Compiler::AssertRegOrConstS(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_s || cf.const_s);
}

void CPU::NewRec::RISCV64Compiler::AssertRegOrConstT(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_t || cf.const_t);
}

#if 0
vixl::aarch64::MemOperand CPU::NewRec::RISCV64Compiler::MipsPtr(Reg r) const
{
  DebugAssert(r < Reg::count);
  return PTR(&g_state.regs.r[static_cast<u32>(r)]);
}
#endif

biscuit::GPR CPU::NewRec::RISCV64Compiler::CFGetRegD(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_d);
  return GPR(cf.host_d);
}

biscuit::GPR CPU::NewRec::RISCV64Compiler::CFGetRegS(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_s);
  return GPR(cf.host_s);
}

biscuit::GPR CPU::NewRec::RISCV64Compiler::CFGetRegT(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_t);
  return GPR(cf.host_t);
}

biscuit::GPR CPU::NewRec::RISCV64Compiler::CFGetRegLO(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_lo);
  return GPR(cf.host_lo);
}

biscuit::GPR CPU::NewRec::RISCV64Compiler::CFGetRegHI(CompileFlags cf) const
{
  DebugAssert(cf.valid_host_hi);
  return GPR(cf.host_hi);
}

void CPU::NewRec::RISCV64Compiler::MoveSToReg(const biscuit::GPR& dst, CompileFlags cf)
{
  if (cf.valid_host_s)
  {
    if (cf.host_s != dst.Index())
      rvAsm->MV(dst, GPR(cf.host_s));
  }
  else if (cf.const_s)
  {
    EmitMov(dst, GetConstantRegU32(cf.MipsS()));
  }
  else
  {
    Log_WarningPrintf("Hit memory path in MoveSToReg() for %s", GetRegName(cf.MipsS()));
    rvAsm->LW(dst, PTR(&g_state.regs.r[cf.mips_s]));
  }
}

void CPU::NewRec::RISCV64Compiler::MoveTToReg(const biscuit::GPR& dst, CompileFlags cf)
{
  if (cf.valid_host_t)
  {
    if (cf.host_t != dst.Index())
      rvAsm->MV(dst, GPR(cf.host_t));
  }
  else if (cf.const_t)
  {
    EmitMov(dst, GetConstantRegU32(cf.MipsT()));
  }
  else
  {
    Log_WarningPrintf("Hit memory path in MoveTToReg() for %s", GetRegName(cf.MipsT()));
    rvAsm->LW(dst, PTR(&g_state.regs.r[cf.mips_t]));
  }
}

void CPU::NewRec::RISCV64Compiler::Flush(u32 flags)
{
  Compiler::Flush(flags);

  if (flags & FLUSH_PC && m_dirty_pc)
  {
    StoreConstantToCPUPointer(m_compiler_pc, &g_state.pc);
    m_dirty_pc = false;
  }

  if (flags & FLUSH_INSTRUCTION_BITS)
  {
    // This sucks, but it's only used for fallbacks.
    Panic("Not implemented");
  }

  if (flags & FLUSH_LOAD_DELAY_FROM_STATE && m_load_delay_dirty)
  {
    // This sucks :(
    // TODO: make it a function?
    rvAsm->LBU(RARG1, PTR(&g_state.load_delay_reg));
    rvAsm->LW(RARG2, PTR(&g_state.load_delay_value));
    rvAsm->SLLI(RARG1, RARG1, 2); // *4
    rvAsm->ADD(RARG1, RARG1, RSTATE);
    rvAsm->SW(RARG2, offsetof(CPU::State, regs.r[0]), RARG1);
    rvAsm->LI(RSCRATCH, static_cast<u8>(Reg::count));
    rvAsm->SB(RSCRATCH, PTR(&g_state.load_delay_reg));
    m_load_delay_dirty = false;
  }

  if (flags & FLUSH_LOAD_DELAY && m_load_delay_register != Reg::count)
  {
    if (m_load_delay_value_register != NUM_HOST_REGS)
      FreeHostReg(m_load_delay_value_register);

    EmitMov(RSCRATCH, static_cast<u8>(m_load_delay_register));
    rvAsm->SB(RSCRATCH, PTR(&g_state.load_delay_reg));
    m_load_delay_register = Reg::count;
    m_load_delay_dirty = true;
  }

  if (flags & FLUSH_GTE_STALL_FROM_STATE && m_dirty_gte_done_cycle)
  {
    // May as well flush cycles while we're here.
    // GTE spanning blocks is very rare, we _could_ disable this for speed.
    rvAsm->LW(RARG1, PTR(&g_state.pending_ticks));
    rvAsm->LW(RARG2, PTR(&g_state.gte_completion_tick));
    if (m_cycles > 0)
    {
      SafeADDI(RARG1, RARG1, m_cycles);
      m_cycles = 0;
    }
    Label no_stall;
    rvAsm->BGE(RARG1, RARG2, &no_stall);
    rvAsm->MV(RARG1, RARG2);
    rvAsm->Bind(&no_stall);
    rvAsm->SW(RARG1, PTR(&g_state.pending_ticks));
    m_dirty_gte_done_cycle = false;
  }

  if (flags & FLUSH_GTE_DONE_CYCLE && m_gte_done_cycle > m_cycles)
  {
    rvAsm->LW(RARG1, PTR(&g_state.pending_ticks));

    // update cycles at the same time
    if (flags & FLUSH_CYCLES && m_cycles > 0)
    {
      SafeADDI(RARG1, RARG1, m_cycles);
      rvAsm->SW(RARG1, PTR(&g_state.pending_ticks));
      m_gte_done_cycle -= m_cycles;
      m_cycles = 0;
    }

    SafeADDI(RARG1, RARG1, m_gte_done_cycle);
    rvAsm->SW(RARG1, PTR(&g_state.gte_completion_tick));
    m_gte_done_cycle = 0;
    m_dirty_gte_done_cycle = true;
  }

  if (flags & FLUSH_CYCLES && m_cycles > 0)
  {
    rvAsm->LW(RARG1, PTR(&g_state.pending_ticks));
    SafeADDI(RARG1, RARG1, m_cycles);
    rvAsm->SW(RARG1, PTR(&g_state.pending_ticks));
    m_gte_done_cycle = std::max<TickCount>(m_gte_done_cycle - m_cycles, 0);
    m_cycles = 0;
  }
}

#if 0

void CPU::NewRec::RISCV64Compiler::Compile_Fallback()
{
  Flush(FLUSH_FOR_INTERPRETER);

#if 0
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
#else
  Panic("Fixme");
#endif
}

#endif

void CPU::NewRec::RISCV64Compiler::CheckBranchTarget(const biscuit::GPR& pcreg)
{
  if (!g_settings.cpu_recompiler_memory_exceptions)
    return;

  DebugAssert(pcreg != RSCRATCH);
  rvAsm->ANDI(RSCRATCH, pcreg, 0x3);
#if 0
  SwitchToFarCode(true, &Assembler::BNEZ, RSCRATCH, zero);
#endif

  BackupHostState();
  EndBlockWithException(Exception::AdEL);

  RestoreHostState();
  SwitchToNearCode(false);
}

void CPU::NewRec::RISCV64Compiler::Compile_jr(CompileFlags cf)
{
  const GPR pcreg = CFGetRegS(cf);
  CheckBranchTarget(pcreg);

  rvAsm->SW(pcreg, PTR(&g_state.pc));

  CompileBranchDelaySlot(false);
  EndBlock(std::nullopt);
}

void CPU::NewRec::RISCV64Compiler::Compile_jalr(CompileFlags cf)
{
  const GPR pcreg = CFGetRegS(cf);
  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetBranchReturnAddress(cf));

  CheckBranchTarget(pcreg);
  rvAsm->SW(pcreg, PTR(&g_state.pc));

  CompileBranchDelaySlot(false);
  EndBlock(std::nullopt);
}

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_bxx(CompileFlags cf, BranchCondition cond)
{
  AssertRegOrConstS(cf);

  const u32 taken_pc = GetConditionalBranchTarget(cf);

  Flush(FLUSH_FOR_BRANCH);

  DebugAssert(cf.valid_host_s);

  // MipsT() here should equal zero for zero branches.
  DebugAssert(cond == BranchCondition::Equal || cond == BranchCondition::NotEqual || cf.MipsT() == Reg::zero);

  Label taken;
  const GPR rs = CFGetRegS(cf);
  switch (cond)
  {
    case BranchCondition::Equal:
    case BranchCondition::NotEqual:
    {
      AssertRegOrConstT(cf);
      if (cf.const_t && HasConstantRegValue(cf.MipsT(), 0))
      {
        (cond == BranchCondition::Equal) ? rvAsm->cbz(rs, &taken) : rvAsm->cbnz(rs, &taken);
      }
      else
      {
        if (cf.valid_host_t)
          rvAsm->cmp(rs, CFGetRegT(cf));
        else if (cf.const_t)
          rvAsm->cmp(rs, armCheckCompareConstant(GetConstantRegU32(cf.MipsT())));

        rvAsm->b(&taken, (cond == BranchCondition::Equal) ? eq : ne);
      }
    }
    break;

    case BranchCondition::GreaterThanZero:
    {
      rvAsm->cmp(rs, 0);
      rvAsm->b(&taken, gt);
    }
    break;

    case BranchCondition::GreaterEqualZero:
    {
      rvAsm->cmp(rs, 0);
      rvAsm->b(&taken, ge);
    }
    break;

    case BranchCondition::LessThanZero:
    {
      rvAsm->cmp(rs, 0);
      rvAsm->b(&taken, lt);
    }
    break;

    case BranchCondition::LessEqualZero:
    {
      rvAsm->cmp(rs, 0);
      rvAsm->b(&taken, le);
    }
    break;
  }

  BackupHostState();
  if (!cf.delay_slot_swapped)
    CompileBranchDelaySlot();

  EndBlock(m_compiler_pc);

  rvAsm->bind(&taken);

  RestoreHostState();
  if (!cf.delay_slot_swapped)
    CompileBranchDelaySlot();

  EndBlock(taken_pc);
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_addi(CompileFlags cf, bool overflow)
{
  const GPR rs = CFGetRegS(cf);
  const GPR rt = CFGetRegT(cf);
  if (const u32 imm = inst->i.imm_sext32(); imm != 0)
  {
    if (!overflow)
    {
      SafeADDI(rt, rs, imm);
    }
    else
    {
      Panic("FIXME");
      // rvAsm->adds(rt, rs, armCheckAddSubConstant(imm));
      TestOverflow(rt);
    }
  }
  else if (rt.Index() != rs.Index())
  {
    rvAsm->MV(rt, rs);
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_addi(CompileFlags cf)
{
  Compile_addi(cf, g_settings.cpu_recompiler_memory_exceptions);
}

void CPU::NewRec::RISCV64Compiler::Compile_addiu(CompileFlags cf)
{
  Compile_addi(cf, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_slti(CompileFlags cf)
{
  Compile_slti(cf, true);
}

void CPU::NewRec::RISCV64Compiler::Compile_sltiu(CompileFlags cf)
{
  Compile_slti(cf, false);
}

#if 0

void CPU::NewRec::RISCV64Compiler::Compile_slti(CompileFlags cf, bool sign)
{
  rvAsm->cmp(CFGetRegS(cf), armCheckCompareConstant(static_cast<s32>(inst->i.imm_sext32())));
  rvAsm->cset(CFGetRegT(cf), sign ? lt : lo);
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_andi(CompileFlags cf)
{
  const GPR rt = CFGetRegT(cf);
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
    SafeANDI(rt, CFGetRegS(cf), imm);
  else
    EmitMov(rt, 0);
}

void CPU::NewRec::RISCV64Compiler::Compile_ori(CompileFlags cf)
{
  const GPR rt = CFGetRegT(cf);
  const GPR rs = CFGetRegS(cf);
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
    SafeORI(rt, rs, imm);
  else if (rt.Index() != rs.Index())
    rvAsm->MV(rt, rs);
}

void CPU::NewRec::RISCV64Compiler::Compile_xori(CompileFlags cf)
{
  const GPR rt = CFGetRegT(cf);
  const GPR rs = CFGetRegS(cf);
  if (const u32 imm = inst->i.imm_zext32(); imm != 0)
    SafeXORI(rt, rs, imm);
  else if (rt.Index() != rs.Index())
    rvAsm->MV(rt, rs);
}

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_shift(CompileFlags cf,
                                                 void (vixl::aarch64::Assembler::*op)(const vixl::aarch64::Register&,
                                                                                      const vixl::aarch64::Register&,
                                                                                      unsigned))
{
  const WRegister rd = CFGetRegD(cf);
  const WRegister rt = CFGetRegT(cf);
  if (inst->r.shamt > 0)
    (rvAsm->*op)(rd, rt, inst->r.shamt);
  else if (rd.GetCode() != rt.GetCode())
    rvAsm->mov(rd, rt);
}

void CPU::NewRec::RISCV64Compiler::Compile_sll(CompileFlags cf)
{
  Compile_shift(cf, &Assembler::lsl);
}

void CPU::NewRec::RISCV64Compiler::Compile_srl(CompileFlags cf)
{
  Compile_shift(cf, &Assembler::lsr);
}

void CPU::NewRec::RISCV64Compiler::Compile_sra(CompileFlags cf)
{
  Compile_shift(cf, &Assembler::asr);
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_variable_shift(
  CompileFlags cf, void (biscuit::Assembler::*op)(biscuit::GPR, biscuit::GPR, biscuit::GPR),
  void (biscuit::Assembler::*op_const)(biscuit::GPR, biscuit::GPR, unsigned))
{
  const GPR rd = CFGetRegD(cf);

  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  const GPR rt = cf.valid_host_t ? CFGetRegT(cf) : RARG2;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  if (cf.const_s)
  {
    if (const u32 shift = GetConstantRegU32(cf.MipsS()); shift != 0)
      (rvAsm->*op_const)(rd, rt, shift & 31u);
    else if (rd.Index() != rt.Index())
      rvAsm->MV(rd, rt);
  }
  else
  {
    (rvAsm->*op)(rd, rt, CFGetRegS(cf));
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_sllv(CompileFlags cf)
{
  Compile_variable_shift(cf, &Assembler::SLLW, &Assembler::SLLIW);
}

void CPU::NewRec::RISCV64Compiler::Compile_srlv(CompileFlags cf)
{
  Compile_variable_shift(cf, &Assembler::SRLW, &Assembler::SRLIW);
}

void CPU::NewRec::RISCV64Compiler::Compile_srav(CompileFlags cf)
{
  Compile_variable_shift(cf, &Assembler::SRAW, &Assembler::SRAIW);
}

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_mult(CompileFlags cf, bool sign)
{
  const GPR rs = cf.valid_host_s ? CFGetRegS(cf) : RARG1;
  if (!cf.valid_host_s)
    MoveSToReg(rs, cf);

  const GPR rt = cf.valid_host_t ? CFGetRegT(cf) : RARG2;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  // TODO: if lo/hi gets killed, we can use a 32-bit multiply
  // This could be problematic if we're not correctly filling in the upper bits..
  const GPR lo = CFGetRegLO(cf);
  const GPR hi = CFGetRegHI(cf);

  if (sign)
  {
    rvAsm->MUL(lo, rs, rt);
    rvAsm->SRAI64(hi, lo, 32);
  }

  (sign) ? rvAsm->MULU(lo.X(), rs, rt) : rvAsm->umull(lo.X(), rs, rt);
  rvAsm->lsr(hi.X(), lo.X(), 32);
}
#endif

void CPU::NewRec::RISCV64Compiler::Compile_mult(CompileFlags cf)
{
  Compile_mult(cf, true);
}

void CPU::NewRec::RISCV64Compiler::Compile_multu(CompileFlags cf)
{
  Compile_mult(cf, false);
}

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_div(CompileFlags cf)
{
  // 36 Volume I: RISC-V User-Level ISA V2.2
  const GPR rs = cf.valid_host_s ? CFGetRegS(cf) : RARG1;
  if (!cf.valid_host_s)
    MoveSToReg(rs, cf);

  const GPR rt = cf.valid_host_t ? CFGetRegT(cf) : RARG2;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  const GPR rlo = CFGetRegLO(cf);
  const GPR rhi = CFGetRegHI(cf);

  Label done;
  Label not_divide_by_zero;
  rvAsm->cbnz(rt, &not_divide_by_zero);
  rvAsm->cmp(rs, 0);
  rvAsm->mov(rhi, rs); // hi = num
  EmitMov(rlo, 1);
  EmitMov(RWSCRATCH, static_cast<u32>(-1));
  rvAsm->csel(rlo, RWSCRATCH, rlo, ge); // lo = s >= 0 ? -1 : 1
  rvAsm->b(&done);

  rvAsm->bind(&not_divide_by_zero);
  Label not_unrepresentable;
  rvAsm->cmp(rs, armCheckCompareConstant(static_cast<s32>(0x80000000u)));
  rvAsm->b(&not_unrepresentable, ne);
  rvAsm->cmp(rt, armCheckCompareConstant(-1));
  rvAsm->b(&not_unrepresentable, ne);

  EmitMov(rlo, 0x80000000u);
  EmitMov(rhi, 0);
  rvAsm->b(&done);

  rvAsm->bind(&not_unrepresentable);

  rvAsm->sdiv(rlo, rs, rt);

  // TODO: skip when hi is dead
  rvAsm->msub(rhi, rlo, rt, rs);

  rvAsm->bind(&done);
}
#endif

void CPU::NewRec::RISCV64Compiler::Compile_divu(CompileFlags cf)
{
  const GPR rs = cf.valid_host_s ? CFGetRegS(cf) : RARG1;
  if (!cf.valid_host_s)
    MoveSToReg(rs, cf);

  const GPR rt = cf.valid_host_t ? CFGetRegT(cf) : RARG2;
  if (!cf.valid_host_t)
    MoveTToReg(rt, cf);

  const GPR rlo = CFGetRegLO(cf);
  const GPR rhi = CFGetRegHI(cf);

  // Semantics match? :-)
  rvAsm->DIVUW(rlo, rs, rt);
  rvAsm->REMUW(rhi, rs, rt);
}

#if 0
void CPU::NewRec::RISCV64Compiler::TestOverflow(const biscuit::GPR& result)
{
  // Volume I: RISC-V User-Level ISA V2.2 13
  SwitchToFarCode(true, vs);

  BackupHostState();

  // toss the result
  ClearHostReg(result.GetCode());

  EndBlockWithException(Exception::Ov);

  RestoreHostState();

  SwitchToNearCode(false);
}

void CPU::NewRec::RISCV64Compiler::Compile_dst_op(CompileFlags cf,
                                                  void (vixl::aarch64::Assembler::*op)(const vixl::aarch64::Register&,
                                                                                       const vixl::aarch64::Register&,
                                                                                       const vixl::aarch64::Operand&),
                                                  bool commutative, bool logical, bool overflow)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  const WRegister rd = CFGetRegD(cf);
  if (cf.valid_host_s && cf.valid_host_t)
  {
    (rvAsm->*op)(rd, CFGetRegS(cf), CFGetRegT(cf));
  }
  else if (commutative && (cf.const_s || cf.const_t))
  {
    const WRegister src = cf.const_s ? CFGetRegT(cf) : CFGetRegS(cf);
    if (const u32 cv = GetConstantRegU32(cf.const_s ? cf.MipsS() : cf.MipsT()); cv != 0)
    {
      (rvAsm->*op)(rd, src, logical ? armCheckLogicalConstant(cv) : armCheckAddSubConstant(cv));
    }
    else
    {
      if (rd.GetCode() != src.GetCode())
        rvAsm->mov(rd, src);
      overflow = false;
    }
  }
  else if (cf.const_s)
  {
    // TODO: Check where we can use wzr here
    EmitMov(RWSCRATCH, GetConstantRegU32(cf.MipsS()));
    (rvAsm->*op)(rd, RWSCRATCH, CFGetRegT(cf));
  }
  else if (cf.const_t)
  {
    const WRegister rs = CFGetRegS(cf);
    if (const u32 cv = GetConstantRegU32(cf.const_s ? cf.MipsS() : cf.MipsT()); cv != 0)
    {
      (rvAsm->*op)(rd, rs, logical ? armCheckLogicalConstant(cv) : armCheckAddSubConstant(cv));
    }
    else
    {
      if (rd.GetCode() != rs.GetCode())
        rvAsm->mov(rd, rs);
      overflow = false;
    }
  }

  if (overflow)
    TestOverflow(rd);
}

void CPU::NewRec::RISCV64Compiler::Compile_add(CompileFlags cf)
{
  if (g_settings.cpu_recompiler_memory_exceptions)
    Compile_dst_op(cf, &Assembler::adds, true, false, true);
  else
    Compile_dst_op(cf, &Assembler::add, true, false, true);
}

void CPU::NewRec::RISCV64Compiler::Compile_addu(CompileFlags cf)
{
  Compile_dst_op(cf, &Assembler::add, true, false, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_sub(CompileFlags cf)
{
  if (g_settings.cpu_recompiler_memory_exceptions)
    Compile_dst_op(cf, &Assembler::subs, false, false, true);
  else
    Compile_dst_op(cf, &Assembler::sub, false, false, true);
}

void CPU::NewRec::RISCV64Compiler::Compile_subu(CompileFlags cf)
{
  Compile_dst_op(cf, &Assembler::sub, false, false, false);
}
#endif

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_and(CompileFlags cf)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  // special cases - and with self -> self, and with 0 -> 0
  const GPR regd = CFGetRegD(cf);
  if (cf.MipsS() == cf.MipsT())
  {
    rvAsm->MV(regd, CFGetRegS(cf));
    return;
  }
  else if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0))
  {
    EmitMov(regd, 0);
    return;
  }

  Compile_dst_op(cf, &Assembler::and_, true, true, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_or(CompileFlags cf)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  // or/nor with 0 -> no effect
  const WRegister regd = CFGetRegD(cf);
  if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0) || cf.MipsS() == cf.MipsT())
  {
    cf.const_s ? MoveTToReg(regd, cf) : MoveSToReg(regd, cf);
    return;
  }

  Compile_dst_op(cf, &Assembler::orr, true, true, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_xor(CompileFlags cf)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  const WRegister regd = CFGetRegD(cf);
  if (cf.MipsS() == cf.MipsT())
  {
    // xor with self -> zero
    rvAsm->mov(regd, wzr);
    return;
  }
  else if (HasConstantRegValue(cf.MipsS(), 0) || HasConstantRegValue(cf.MipsT(), 0))
  {
    // xor with zero -> no effect
    cf.const_s ? MoveTToReg(regd, cf) : MoveSToReg(regd, cf);
    return;
  }

  Compile_dst_op(cf, &Assembler::eor, true, true, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_nor(CompileFlags cf)
{
  Compile_or(cf);
  rvAsm->mvn(CFGetRegD(cf), CFGetRegD(cf));
}

void CPU::NewRec::RISCV64Compiler::Compile_slt(CompileFlags cf)
{
  Compile_slt(cf, true);
}

void CPU::NewRec::RISCV64Compiler::Compile_sltu(CompileFlags cf)
{
  Compile_slt(cf, false);
}

void CPU::NewRec::RISCV64Compiler::Compile_slt(CompileFlags cf, bool sign)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);

  // TODO: swap and reverse op for constants
  if (cf.const_s)
  {
    EmitMov(RWSCRATCH, GetConstantRegS32(cf.MipsS()));
    rvAsm->cmp(RWSCRATCH, CFGetRegT(cf));
  }
  else if (cf.const_t)
  {
    rvAsm->cmp(CFGetRegS(cf), armCheckCompareConstant(GetConstantRegS32(cf.MipsT())));
  }
  else
  {
    rvAsm->cmp(CFGetRegS(cf), CFGetRegT(cf));
  }

  rvAsm->cset(CFGetRegD(cf), sign ? lt : lo);
}
#endif

void CPU::NewRec::RISCV64Compiler::FlushForLoadStore(const std::optional<VirtualMemoryAddress>& address, bool store)
{
  if (g_settings.IsUsingFastmem())
    return;

  // TODO: Stores don't need to flush GTE cycles...
  Flush(FLUSH_FOR_C_CALL | FLUSH_FOR_LOADSTORE);
}

biscuit::GPR CPU::NewRec::RISCV64Compiler::ComputeLoadStoreAddressArg(
  CompileFlags cf, const std::optional<VirtualMemoryAddress>& address, const std::optional<const biscuit::GPR>& reg)
{
  const u32 imm = inst->i.imm_sext32();
  if (cf.valid_host_s && imm == 0 && !reg.has_value())
    return CFGetRegS(cf);

  const GPR dst = reg.has_value() ? reg.value() : RARG1;
  if (address.has_value())
  {
    EmitMov(dst, address.value());
  }
  else if (imm == 0)
  {
    if (cf.valid_host_s)
    {
      if (const GPR src = CFGetRegS(cf); src.Index() != dst.Index())
        rvAsm->MV(dst, CFGetRegS(cf));
    }
    else
    {
      rvAsm->LW(dst, PTR(&g_state.regs.r[cf.mips_s]));
    }
  }
  else
  {
    if (cf.valid_host_s)
    {
      SafeADDI(dst, CFGetRegS(cf), inst->i.imm_sext32());
    }
    else
    {
      rvAsm->LW(dst, PTR(&g_state.regs.r[cf.mips_s]));
      SafeADDI(dst, dst, inst->i.imm_sext32());
    }
  }

  return dst;
}

#if 0
template<typename RegAllocFn>
void CPU::NewRec::RISCV64Compiler::GenerateLoad(const biscuit::GPR& addr_reg, MemoryAccessSize size, bool sign,
                                                const RegAllocFn& dst_reg_alloc)
{
  const bool checked = g_settings.cpu_recompiler_memory_exceptions;
  if (!checked && g_settings.IsUsingFastmem())
  {
    m_cycles += Bus::RAM_READ_TICKS;

    const WRegister dst = dst_reg_alloc();
    rvAsm->ADD(RSCRATCH, RMEMBASE, addr_reg);
    u8* start = m_emitter->GetCursorPointer();
    switch (size)
    {
      case MemoryAccessSize::Byte:
        sign ? rvAsm->LB(dst, 0, RSCRATCH) : rvAsm->LBU(dst, 0, RSCRATCH);
        break;

      case MemoryAccessSize::HalfWord:
        sign ? rvAsm->LH(dst, 0, RSCRATCH) : rvAsm->LHU(dst, 0, RSCRATCH);
        break;

      case MemoryAccessSize::Word:
        rvAsm->LW(dst, 0, RSCRATCH);
        break;
    }

    AddLoadStoreInfo(start, 4, addr_reg.Index(), dst.Index(), size, sign, true);
    return;
  }

  if (addr_reg.Index() != RARG1.Index())
    rvAsm->mov(RARG1, addr_reg);

  switch (size)
  {
    case MemoryAccessSize::Byte:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::ReadMemoryByte) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryByte));
    }
    break;
    case MemoryAccessSize::HalfWord:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::ReadMemoryHalfWord) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryHalfWord));
    }
    break;
    case MemoryAccessSize::Word:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::ReadMemoryWord) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryWord));
    }
    break;
  }

  // TODO: turn this into an asm function instead
  if (checked)
  {
    SwitchToFarCodeIfBitSet(RXRET, 63);
    BackupHostState();

    // Need to stash this in a temp because of the flush.
    const WRegister temp = WRegister(AllocateTempHostReg(HR_CALLEE_SAVED));
    rvAsm->neg(temp.X(), RXRET);
    rvAsm->lsl(temp, temp, 2);

    Flush(FLUSH_FOR_C_CALL | FLUSH_FLUSH_MIPS_REGISTERS | FLUSH_FOR_EXCEPTION);

    // cause_bits = (-result << 2) | BD | cop_n
    rvAsm->orr(RWARG1, temp,
               armCheckLogicalConstant(Cop0Registers::CAUSE::MakeValueForException(
                 static_cast<Exception>(0), m_current_instruction_branch_delay_slot, false, inst->cop.cop_n)));
    EmitMov(RWARG2, m_current_instruction_pc);
    EmitCall(reinterpret_cast<const void*>(static_cast<void (*)(u32, u32)>(&CPU::RaiseException)));
    FreeHostReg(temp.GetCode());
    EndBlock(std::nullopt);

    RestoreHostState();
    SwitchToNearCode(false);
  }

  const WRegister dst_reg = dst_reg_alloc();
  switch (size)
  {
    case MemoryAccessSize::Byte:
    {
      sign ? rvAsm->sxtb(dst_reg, RWRET) : rvAsm->uxtb(dst_reg, RWRET);
    }
    break;
    case MemoryAccessSize::HalfWord:
    {
      sign ? rvAsm->sxth(dst_reg, RWRET) : rvAsm->uxth(dst_reg, RWRET);
    }
    break;
    case MemoryAccessSize::Word:
    {
      if (dst_reg.GetCode() != RWRET.GetCode())
        rvAsm->mov(dst_reg, RWRET);
    }
    break;
  }
}

void CPU::NewRec::RISCV64Compiler::GenerateStore(const biscuit::GPR& addr_reg, const biscuit::GPR& value_reg,
                                                 MemoryAccessSize size)
{
  const bool checked = g_settings.cpu_recompiler_memory_exceptions;
  if (!checked && g_settings.IsUsingFastmem())
  {
    DebugAssert(value_reg != RSCRATCH);
    rvAsm->ADD(RSCRATCH, RMEMBASE, addr_reg);
    u8* start = m_emitter->GetCursorPointer();
    switch (size)
    {
      case MemoryAccessSize::Byte:
        rvAsm->SB(value_reg, 0, RSCRATCH);
        break;

      case MemoryAccessSize::HalfWord:
        rvAsm->SH(value_reg, 0, RSCRATCH);
        break;

      case MemoryAccessSize::Word:
        rvAsm->SW(value_reg, 0, RSCRATCH);
        break;
    }
    AddLoadStoreInfo(start, 4, addr_reg.Index(), value_reg.Index(), size, false, false);
    return;
  }

  if (addr_reg.Index() != RARG1.Index())
    rvAsm->MV(RARG1, addr_reg);
  if (value_reg.Index() != RARG2.Index())
    rvAsm->MV(RARG2, value_reg);

  switch (size)
  {
    case MemoryAccessSize::Byte:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::WriteMemoryByte) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryByte));
    }
    break;
    case MemoryAccessSize::HalfWord:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::WriteMemoryHalfWord) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryHalfWord));
    }
    break;
    case MemoryAccessSize::Word:
    {
      EmitCall(checked ? reinterpret_cast<const void*>(&Recompiler::Thunks::WriteMemoryWord) :
                         reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryWord));
    }
    break;
  }

  // TODO: turn this into an asm function instead
  if (checked)
  {
    SwitchToFarCodeIfRegZeroOrNonZero(RXRET, true);
    BackupHostState();

    // Need to stash this in a temp because of the flush.
    const WRegister temp = WRegister(AllocateTempHostReg(HR_CALLEE_SAVED));
    rvAsm->lsl(temp, RWRET, 2);

    Flush(FLUSH_FOR_C_CALL | FLUSH_FLUSH_MIPS_REGISTERS | FLUSH_FOR_EXCEPTION);

    // cause_bits = (result << 2) | BD | cop_n
    rvAsm->orr(RWARG1, temp,
               armCheckLogicalConstant(Cop0Registers::CAUSE::MakeValueForException(
                 static_cast<Exception>(0), m_current_instruction_branch_delay_slot, false, inst->cop.cop_n)));
    EmitMov(RWARG2, m_current_instruction_pc);
    EmitCall(reinterpret_cast<const void*>(static_cast<void (*)(u32, u32)>(&CPU::RaiseException)));
    FreeHostReg(temp.GetCode());
    EndBlock(std::nullopt);

    RestoreHostState();
    SwitchToNearCode(false);
  }
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_lxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                               const std::optional<VirtualMemoryAddress>& address)
{
  FlushForLoadStore(address, false);
  const GPR addr = ComputeLoadStoreAddressArg(cf, address);
  GenerateLoad(addr, size, sign, [this, cf]() {
    if (cf.MipsT() == Reg::zero)
      return RRET;

    return GPR(AllocateHostReg(HR_MODE_WRITE, EMULATE_LOAD_DELAYS ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG,
                               cf.MipsT()));
  });
}

#if 0

void CPU::NewRec::RISCV64Compiler::Compile_lwx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                               const std::optional<VirtualMemoryAddress>& address)
{
  DebugAssert(size == MemoryAccessSize::Word && !sign);
  FlushForLoadStore(address, false);

  // TODO: if address is constant, this can be simplified..

  // If we're coming from another block, just flush the load delay and hope for the best..
  if (m_load_delay_dirty)
    UpdateLoadDelay();

  // We'd need to be careful here if we weren't overwriting it..
  const WRegister addr = WRegister(AllocateHostReg(HR_CALLEE_SAVED, HR_TYPE_TEMP));
  ComputeLoadStoreAddressArg(cf, address, addr);
  rvAsm->and_(RWARG1, addr, armCheckLogicalConstant(~0x3u));
  GenerateLoad(RWARG1, MemoryAccessSize::Word, false, []() { return RWRET; });

  if (inst->r.rt == Reg::zero)
  {
    FreeHostReg(addr.GetCode());
    return;
  }

  // lwl/lwr from a load-delayed value takes the new value, but it itself, is load delayed, so the original value is
  // never written back. NOTE: can't trust T in cf because of the flush
  const Reg rt = inst->r.rt;
  WRegister value;
  if (m_load_delay_register == rt)
  {
    const u32 existing_ld_rt = (m_load_delay_value_register == NUM_HOST_REGS) ?
                                 AllocateHostReg(HR_MODE_READ, HR_TYPE_LOAD_DELAY_VALUE, rt) :
                                 m_load_delay_value_register;
    RenameHostReg(existing_ld_rt, HR_MODE_WRITE, HR_TYPE_NEXT_LOAD_DELAY_VALUE, rt);
    value = WRegister(existing_ld_rt);
  }
  else
  {
    if constexpr (EMULATE_LOAD_DELAYS)
    {
      value = WRegister(AllocateHostReg(HR_MODE_WRITE, HR_TYPE_NEXT_LOAD_DELAY_VALUE, rt));
      if (const std::optional<u32> rtreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt); rtreg.has_value())
        rvAsm->mov(value, WRegister(rtreg.value()));
      else if (HasConstantReg(rt))
        EmitMov(value, GetConstantRegU32(rt));
      else
        rvAsm->ldr(value, MipsPtr(rt));
    }
    else
    {
      value = WRegister(AllocateHostReg(HR_MODE_READ | HR_MODE_WRITE, HR_TYPE_CPU_REG, rt));
    }
  }

  DebugAssert(value.GetCode() != RWARG2.GetCode() && value.GetCode() != RWARG3.GetCode());
  rvAsm->and_(RWARG2, addr, 3);
  rvAsm->lsl(RWARG2, RWARG2, 3); // *8
  EmitMov(RWARG3, 24);
  rvAsm->sub(RWARG3, RWARG3, RWARG2);

  if (inst->op == InstructionOp::lwl)
  {
    // const u32 mask = UINT32_C(0x00FFFFFF) >> shift;
    // new_value = (value & mask) | (RWRET << (24 - shift));
    EmitMov(addr, 0xFFFFFFu);
    rvAsm->lsrv(addr, addr, RWARG2);
    rvAsm->and_(value, value, addr);
    rvAsm->lslv(RWRET, RWRET, RWARG3);
    rvAsm->orr(value, value, RWRET);
  }
  else
  {
    // const u32 mask = UINT32_C(0xFFFFFF00) << (24 - shift);
    // new_value = (value & mask) | (RWRET >> shift);
    rvAsm->lsrv(RWRET, RWRET, RWARG2);
    EmitMov(addr, 0xFFFFFF00u);
    rvAsm->lslv(addr, addr, RWARG3);
    rvAsm->and_(value, value, addr);
    rvAsm->orr(value, value, RWRET);
  }

  FreeHostReg(addr.GetCode());
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_lwc2(CompileFlags cf, MemoryAccessSize size, bool sign,
                                                const std::optional<VirtualMemoryAddress>& address)
{
  FlushForLoadStore(address, false);
  const GPR addr = ComputeLoadStoreAddressArg(cf, address);
  GenerateLoad(addr, MemoryAccessSize::Word, false, []() { return RRET; });

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
      rvAsm->SW(RRET, PTR(ptr));
      return;
    }

    case GTERegisterAccessAction::SignExtend16:
    {
      SExtH(RRET, RRET);
      rvAsm->SW(RRET, PTR(ptr));
      return;
    }

    case GTERegisterAccessAction::ZeroExtend16:
    {
      UExtH(RRET, RRET);
      rvAsm->SW(RRET, PTR(ptr));
      return;
    }

    case GTERegisterAccessAction::CallHandler:
    {
      Flush(FLUSH_FOR_C_CALL);
      rvAsm->MV(RARG2, RRET);
      EmitMov(RARG1, index);
      EmitCall(reinterpret_cast<const void*>(&GTE::WriteRegister));
      return;
    }

    case GTERegisterAccessAction::PushFIFO:
    {
      // SXY0 <- SXY1
      // SXY1 <- SXY2
      // SXY2 <- SXYP
      DebugAssert(RRET.Index() != RARG2.Index() && RRET.Index() != RARG3.Index());
      rvAsm->LW(RARG2, PTR(&g_state.gte_regs.SXY1[0]));
      rvAsm->LW(RARG3, PTR(&g_state.gte_regs.SXY2[0]));
      rvAsm->SW(RARG2, PTR(&g_state.gte_regs.SXY0[0]));
      rvAsm->SW(RARG3, PTR(&g_state.gte_regs.SXY1[0]));
      rvAsm->SW(RRET, PTR(&g_state.gte_regs.SXY2[0]));
      return;
    }

    default:
    {
      Panic("Unknown action");
      return;
    }
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_sxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                               const std::optional<VirtualMemoryAddress>& address)
{
  AssertRegOrConstS(cf);
  AssertRegOrConstT(cf);
  FlushForLoadStore(address, true);
  const GPR addr = ComputeLoadStoreAddressArg(cf, address);

  if (!cf.valid_host_t)
    MoveTToReg(RARG2, cf);

  GenerateStore(addr, cf.valid_host_t ? CFGetRegT(cf) : RARG2, size);
}

#if 0

void CPU::NewRec::RISCV64Compiler::Compile_swx(CompileFlags cf, MemoryAccessSize size, bool sign,
                                               const std::optional<VirtualMemoryAddress>& address)
{
  DebugAssert(size == MemoryAccessSize::Word && !sign);
  FlushForLoadStore(address, true);

  // TODO: if address is constant, this can be simplified..
  // We'd need to be careful here if we weren't overwriting it..
  const WRegister addr = WRegister(AllocateHostReg(HR_CALLEE_SAVED, HR_TYPE_TEMP));
  ComputeLoadStoreAddressArg(cf, address, addr);
  rvAsm->and_(RWARG1, addr, armCheckLogicalConstant(~0x3u));
  GenerateLoad(RWARG1, MemoryAccessSize::Word, false, []() { return RWRET; });

  // TODO: this can take over rt's value if it's no longer needed
  // NOTE: can't trust T in cf because of the flush
  const Reg rt = inst->r.rt;
  const WRegister value = RWARG2;
  if (const std::optional<u32> rtreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt); rtreg.has_value())
    rvAsm->mov(value, WRegister(rtreg.value()));
  else if (HasConstantReg(rt))
    EmitMov(value, GetConstantRegU32(rt));
  else
    rvAsm->ldr(value, MipsPtr(rt));

  rvAsm->and_(RWSCRATCH, addr, 3);
  rvAsm->lsl(RWSCRATCH, RWSCRATCH, 3); // *8

  if (inst->op == InstructionOp::swl)
  {
    // const u32 mem_mask = UINT32_C(0xFFFFFF00) << shift;
    // new_value = (RWRET & mem_mask) | (value >> (24 - shift));
    EmitMov(RWARG3, 0xFFFFFF00u);
    rvAsm->lslv(RWARG3, RWARG3, RWSCRATCH);
    rvAsm->and_(RWRET, RWRET, RWARG3);

    EmitMov(RWARG3, 24);
    rvAsm->sub(RWARG3, RWARG3, RWSCRATCH);
    rvAsm->lsrv(value, value, RWARG3);
    rvAsm->orr(value, value, RWRET);
  }
  else
  {
    // const u32 mem_mask = UINT32_C(0x00FFFFFF) >> (24 - shift);
    // new_value = (RWRET & mem_mask) | (value << shift);
    rvAsm->lslv(value, value, RWSCRATCH);

    EmitMov(RWARG3, 24);
    rvAsm->sub(RWARG3, RWARG3, RWSCRATCH);
    EmitMov(RWSCRATCH, 0x00FFFFFFu);
    rvAsm->lsrv(RWSCRATCH, RWSCRATCH, RWARG3);
    rvAsm->and_(RWRET, RWRET, RWSCRATCH);
    rvAsm->orr(value, value, RWRET);
  }

  FreeHostReg(addr.GetCode());

  rvAsm->and_(RWARG1, addr, armCheckLogicalConstant(~0x3u));
  GenerateStore(RWARG1, value, MemoryAccessSize::Word);
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_swc2(CompileFlags cf, MemoryAccessSize size, bool sign,
                                                const std::optional<VirtualMemoryAddress>& address)
{
  FlushForLoadStore(address, true);

  const u32 index = static_cast<u32>(inst->r.rt.GetValue());
  const auto [ptr, action] = GetGTERegisterPointer(index, false);
  switch (action)
  {
    case GTERegisterAccessAction::Direct:
    {
      rvAsm->LW(RARG2, PTR(ptr));
    }
    break;

    case GTERegisterAccessAction::CallHandler:
    {
      // should already be flushed.. except in fastmem case
      Flush(FLUSH_FOR_C_CALL);
      EmitMov(RARG1, index);
      EmitCall(reinterpret_cast<const void*>(&GTE::ReadRegister));
      rvAsm->MV(RARG2, RRET);
    }
    break;

    default:
    {
      Panic("Unknown action");
    }
    break;
  }

  const GPR addr = ComputeLoadStoreAddressArg(cf, address);
  GenerateStore(addr, RARG2, size);
}

#if 0
void CPU::NewRec::RISCV64Compiler::Compile_mtc0(CompileFlags cf)
{
  // TODO: we need better constant setting here.. which will need backprop
  AssertRegOrConstT(cf);

  const Cop0Reg reg = static_cast<Cop0Reg>(MipsD());
  const u32* ptr = GetCop0RegPtr(reg);
  const u32 mask = GetCop0RegWriteMask(reg);
  if (!ptr)
  {
    Compile_Fallback();
    return;
  }

  if (mask == 0)
  {
    // if it's a read-only register, ignore
    Log_DebugPrintf("Ignoring write to read-only cop0 reg %u", static_cast<u32>(reg));
    return;
  }

  // for some registers, we need to test certain bits
  const bool needs_bit_test = (reg == Cop0Reg::SR && g_settings.IsUsingFastmem());
  const WRegister new_value = RWARG1;
  const WRegister old_value = RWARG2;
  const WRegister changed_bits = RWARG3;
  const WRegister mask_reg = RWSCRATCH;

  // Load old value
  rvAsm->ldr(old_value, PTR(ptr));

  // No way we fit this in an immediate..
  EmitMov(mask_reg, mask);

  // update value
  if (cf.valid_host_t)
    rvAsm->and_(new_value, CFGetRegT(cf), mask_reg);
  else
    EmitMov(new_value, GetConstantRegU32(cf.MipsT()) & mask);

  if (needs_bit_test)
    rvAsm->eor(changed_bits, old_value, new_value);
  rvAsm->bic(old_value, old_value, mask_reg);
  rvAsm->orr(new_value, old_value, new_value);
  rvAsm->str(new_value, PTR(ptr));

  if (reg == Cop0Reg::SR && g_settings.IsUsingFastmem())
  {
    // TODO: replace with register backup
    // We could just inline the whole thing..
    Flush(FLUSH_FOR_C_CALL);

    SwitchToFarCodeIfBitSet(changed_bits, 16);
    rvAsm->sub(sp, sp, 16);
    rvAsm->stp(RWARG1, RWARG2, MemOperand(sp));
    EmitCall(reinterpret_cast<const void*>(&CPU::UpdateFastmemBase));
    rvAsm->ldp(RWARG1, RWARG2, MemOperand(sp));
    rvAsm->add(sp, sp, 16);
    rvAsm->ldr(RMEMBASE, PTR(&g_state.fastmem_base));
    SwitchToNearCode(true);
  }

  if (reg == Cop0Reg::SR || reg == Cop0Reg::CAUSE)
  {
    const WRegister sr = (reg == Cop0Reg::SR) ? RWARG2 : (rvAsm->ldr(RWARG1, PTR(&g_state.cop0_regs.sr.bits)), RWARG1);
    TestInterrupts(sr);
  }

  if (reg == Cop0Reg::DCIC && g_settings.cpu_recompiler_memory_exceptions)
  {
    // TODO: DCIC handling for debug breakpoints
    Log_WarningPrintf("TODO: DCIC handling for debug breakpoints");
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_rfe(CompileFlags cf)
{
  // shift mode bits right two, preserving upper bits
  rvAsm->ldr(RWARG1, PTR(&g_state.cop0_regs.sr.bits));
  rvAsm->ubfx(RWARG2, RWARG1, 2, 4);
  rvAsm->bfi(RWARG1, RWARG2, 0, 4);
  rvAsm->str(RWARG1, PTR(&g_state.cop0_regs.sr.bits));

  TestInterrupts(RWARG1);
}

void CPU::NewRec::RISCV64Compiler::TestInterrupts(const biscuit::GPR& sr)
{
  // if Iec == 0 then goto no_interrupt
  Label no_interrupt;
  rvAsm->tbz(sr, 0, &no_interrupt);

  // sr & cause
  rvAsm->ldr(RWSCRATCH, PTR(&g_state.cop0_regs.cause.bits));
  rvAsm->and_(sr, sr, RWSCRATCH);

  // ((sr & cause) & 0xff00) == 0 goto no_interrupt
  rvAsm->tst(sr, 0xFF00);

  SwitchToFarCode(true, ne);
  BackupHostState();
  Flush(FLUSH_FLUSH_MIPS_REGISTERS | FLUSH_FOR_EXCEPTION | FLUSH_FOR_C_CALL);
  EmitCall(reinterpret_cast<const void*>(&DispatchInterrupt));
  EndBlock(std::nullopt);
  RestoreHostState();
  SwitchToNearCode(false);

  rvAsm->bind(&no_interrupt);
}

#endif

void CPU::NewRec::RISCV64Compiler::Compile_mfc2(CompileFlags cf)
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
    rvAsm->LW(GPR(hreg), PTR(ptr));
  }
  else if (action == GTERegisterAccessAction::CallHandler)
  {
    Flush(FLUSH_FOR_C_CALL);
    EmitMov(RARG1, index);
    EmitCall(reinterpret_cast<const void*>(&GTE::ReadRegister));

    const u32 hreg =
      AllocateHostReg(HR_MODE_WRITE, EMULATE_LOAD_DELAYS ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG, rt);
    rvAsm->MV(GPR(hreg), RRET);
  }
  else
  {
    Panic("Unknown action");
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_mtc2(CompileFlags cf)
{
  const u32 index = inst->cop.Cop2Index();
  const auto [ptr, action] = GetGTERegisterPointer(index, true);
  if (action == GTERegisterAccessAction::Ignore)
    return;

  if (action == GTERegisterAccessAction::Direct)
  {
    if (cf.const_t)
      StoreConstantToCPUPointer(GetConstantRegU32(cf.MipsT()), ptr);
    else
      rvAsm->SW(CFGetRegT(cf), PTR(ptr));
  }
  else if (action == GTERegisterAccessAction::SignExtend16 || action == GTERegisterAccessAction::ZeroExtend16)
  {
    const bool sign = (action == GTERegisterAccessAction::SignExtend16);
    if (cf.valid_host_t)
    {
      sign ? SExtH(RARG1, CFGetRegT(cf)) : UExtH(RARG1, CFGetRegT(cf));
      rvAsm->SW(RARG1, PTR(ptr));
    }
    else if (cf.const_t)
    {
      const u16 cv = Truncate16(GetConstantRegU32(cf.MipsT()));
      StoreConstantToCPUPointer(sign ? ::SignExtend32(cv) : ::ZeroExtend32(cv), ptr);
    }
    else
    {
      Panic("Unsupported setup");
    }
  }
  else if (action == GTERegisterAccessAction::CallHandler)
  {
    Flush(FLUSH_FOR_C_CALL);
    EmitMov(RARG1, index);
    MoveTToReg(RARG2, cf);
    EmitCall(reinterpret_cast<const void*>(&GTE::WriteRegister));
  }
  else if (action == GTERegisterAccessAction::PushFIFO)
  {
    // SXY0 <- SXY1
    // SXY1 <- SXY2
    // SXY2 <- SXYP
    DebugAssert(RRET.Index() != RARG2.Index() && RRET.Index() != RARG3.Index());
    rvAsm->LW(RARG2, PTR(&g_state.gte_regs.SXY1[0]));
    rvAsm->LW(RARG3, PTR(&g_state.gte_regs.SXY2[0]));
    rvAsm->SW(RARG2, PTR(&g_state.gte_regs.SXY0[0]));
    rvAsm->SW(RARG3, PTR(&g_state.gte_regs.SXY1[0]));
    if (cf.valid_host_t)
      rvAsm->SW(CFGetRegT(cf), PTR(&g_state.gte_regs.SXY2[0]));
    else if (cf.const_t)
      StoreConstantToCPUPointer(GetConstantRegU32(cf.MipsT()), &g_state.gte_regs.SXY2[0]);
    else
      Panic("Unsupported setup");
  }
  else
  {
    Panic("Unknown action");
  }
}

void CPU::NewRec::RISCV64Compiler::Compile_cop2(CompileFlags cf)
{
  TickCount func_ticks;
  GTE::InstructionImpl func = GTE::GetInstructionImpl(inst->bits, &func_ticks);

  Flush(FLUSH_FOR_C_CALL);
  EmitMov(RARG1, inst->bits & GTE::Instruction::REQUIRED_BITS_MASK);
  EmitCall(reinterpret_cast<const void*>(func));

  AddGTETicks(func_ticks);
}

void CPU::NewRec::RISCV64Compiler::SExtH(const biscuit::GPR& rd, const biscuit::GPR& rs)
{
  rvAsm->SLLI(rd, rs, 16);
  rvAsm->SRAIW(rd, rd, 16);
}

void CPU::NewRec::RISCV64Compiler::UExtH(const biscuit::GPR& rd, const biscuit::GPR& rs)
{
  rvAsm->SLLI(rd, rs, 16);
  rvAsm->SRLI(rd, rd, 16);
}

#if 0

u32 CPU::NewRec::CompileASMFunctions(u8* code, u32 code_size)
{
  Assembler actual_asm(code, code_size);
  Assembler* armAsm = &actual_asm;

#ifdef VIXL_DEBUG
  vixl::CodeBufferCheckScope asm_check(armAsm, code_size, vixl::CodeBufferCheckScope::kDontReserveBufferSpace);
#endif

  Label dispatch;

  g_enter_recompiler = armAsm->GetCursorAddress<const void*>();
  {
    // Save all callee-saved regs so we don't need to.
    // TODO: reserve some space for saving caller-saved registers
    armAsm->sub(sp, sp, 144);
    armAsm->stp(x19, x20, MemOperand(sp, 32));
    armAsm->stp(x21, x22, MemOperand(sp, 48));
    armAsm->stp(x23, x24, MemOperand(sp, 64));
    armAsm->stp(x25, x26, MemOperand(sp, 80));
    armAsm->stp(x27, x28, MemOperand(sp, 96));
    armAsm->stp(x29, lr, MemOperand(sp, 112));

    // Need the CPU state for basically everything :-)
    armMoveAddressToReg(armAsm, RSTATE, &g_state);

    // Fastmem setup
    if (g_settings.IsUsingFastmem())
      armAsm->ldr(RMEMBASE, PTR(&g_state.fastmem_base));

    // Downcount isn't set on entry, so we need to initialize it
    armMoveAddressToReg(armAsm, RXARG1, TimingEvents::GetHeadEventPtr());
    armAsm->ldr(RXARG1, MemOperand(RXARG1));
    armAsm->ldr(RWARG1, MemOperand(RXARG1, offsetof(TimingEvent, m_downcount)));
    armAsm->str(RWARG1, PTR(&g_state.downcount));

    // Fall through to event dispatcher
  }

  // check events then for frame done
  g_check_events_and_dispatch = armAsm->GetCursorAddress<const void*>();
  {
    Label update_downcount, check_interrupts;
    armMoveAddressToReg(armAsm, RXARG1, TimingEvents::GetHeadEventPtr());
    armAsm->ldr(RXARG1, MemOperand(RXARG1));
    armAsm->ldr(RWARG1, MemOperand(RXARG1, offsetof(TimingEvent, m_downcount)));
    armAsm->ldr(RWARG2, PTR(&g_state.pending_ticks));
    armAsm->cmp(RWARG1, RWARG2);
    armAsm->b(&update_downcount, gt);
    armEmitCall(armAsm, reinterpret_cast<const void*>(&TimingEvents::RunEvents), true);
    armAsm->b(&check_interrupts);

    // TODO: this _shouldn't_ be necessary, because if we're flagging IRQ, then downcount should get restored.
    armAsm->bind(&update_downcount);
    armAsm->ldr(RWARG1, PTR(&g_state.downcount));

    armAsm->bind(&check_interrupts);

    // eax <- sr
    armAsm->ldr(RWARG1, PTR(&g_state.cop0_regs.sr.bits));

    // if Iec == 0 then goto no_interrupt
    armAsm->tbz(RWARG1, 0, &dispatch);

    // sr & cause
    armAsm->ldr(RWARG2, PTR(&g_state.cop0_regs.cause.bits));
    armAsm->and_(RWARG1, RWARG1, RWARG2);

    // ((sr & cause) & 0xff00) == 0 goto no_interrupt
    armAsm->tst(RWARG1, 0xFF00);
    armAsm->b(&dispatch, eq);

    // we have an interrupt
    armEmitCall(armAsm, reinterpret_cast<const void*>(&DispatchInterrupt), true);
  }

  // TODO: align?
  g_dispatcher = armAsm->GetCursorAddress<const void*>();
  {
    armAsm->bind(&dispatch);

    // x9 <- s_fast_map[pc >> 16]
    armAsm->ldr(RWARG1, PTR(&g_state.pc));
    armMoveAddressToReg(armAsm, RXARG3, g_fast_map.data());
    armAsm->lsr(RWARG2, RWARG1, 16);
    armAsm->lsr(RWARG1, RWARG1, 2);
    armAsm->ldr(RXARG2, MemOperand(RXARG3, RXARG2, LSL, 3));

    // blr(x9[pc * 2]) (fast_map[pc >> 2])
    armAsm->ldr(RXARG1, MemOperand(RXARG2, RXARG1, LSL, 3));
    armAsm->blr(RXARG1);
  }

  g_compile_or_revalidate_block = armAsm->GetCursorAddress<const void*>();
  {
    armAsm->ldr(RWARG1, PTR(&g_state.pc));
    armEmitCall(armAsm, reinterpret_cast<const void*>(&CompileOrRevalidateBlock), true);
    armAsm->b(&dispatch);
  }

  g_exit_recompiler = armAsm->GetCursorAddress<const void*>();
  {
    armAsm->ldp(x29, lr, MemOperand(sp, 112));
    armAsm->ldp(x27, x28, MemOperand(sp, 96));
    armAsm->ldp(x25, x26, MemOperand(sp, 80));
    armAsm->ldp(x23, x24, MemOperand(sp, 64));
    armAsm->ldp(x21, x22, MemOperand(sp, 48));
    armAsm->ldp(x19, x20, MemOperand(sp, 32));
    armAsm->add(sp, sp, 144);
    armAsm->ret();
  }

  armAsm->FinalizeCode();

  // TODO: align?
  s_trampoline_targets.clear();
  s_trampoline_start_ptr = code + armAsm->GetCursorOffset();
  s_trampoline_used = 0;

  return static_cast<u32>(armAsm->GetCursorOffset()) + TRAMPOLINE_AREA_SIZE;
}

u32 CPU::NewRec::EmitJump(void* code, const void* dst, bool flush_icache)
{
  const s64 disp = armGetPCDisplacement(code, dst);
  DebugAssert(vixl::IsInt26(disp));

  const u32 new_code = B | Assembler::ImmUncondBranch(disp);
  std::memcpy(code, &new_code, sizeof(new_code));
  if (flush_icache)
    rvFlushInstructionCache(code, kInstructionSize);

  return kInstructionSize;
}

u32 CPU::NewRec::BackpatchLoadStore(void* thunk_code, u32 thunk_space, void* code_address, u32 code_size,
                                    TickCount cycles_to_add, TickCount cycles_to_remove, u32 gpr_bitmask,
                                    u8 address_register, u8 data_register, MemoryAccessSize size, bool is_signed,
                                    bool is_load)
{
  Assembler arm_asm(static_cast<u8*>(thunk_code), thunk_space);
  Assembler* armAsm = &arm_asm;

#ifdef VIXL_DEBUG
  vixl::CodeBufferCheckScope asm_check(armAsm, thunk_space, vixl::CodeBufferCheckScope::kDontReserveBufferSpace);
#endif

  static constexpr u32 GPR_SIZE = 8;

  // save regs
  u32 num_gprs = 0;

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    if ((gpr_bitmask & (1u << i)) && rvIsCallerSavedRegister(i) && (!is_load || data_register != i))
      num_gprs++;
  }

  const u32 stack_size = (((num_gprs + 1) & ~1u) * GPR_SIZE);

  // TODO: use stp+ldp, vixl helper?

  if (stack_size > 0)
  {
    armAsm->sub(sp, sp, stack_size);

    u32 stack_offset = 0;
    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      if ((gpr_bitmask & (1u << i)) && rvIsCallerSavedRegister(i) && (!is_load || data_register != i))
      {
        armAsm->str(XRegister(i), MemOperand(sp, stack_offset));
        stack_offset += GPR_SIZE;
      }
    }
  }

  if (cycles_to_add != 0)
  {
    // NOTE: we have to reload here, because memory writes can run DMA, which can screw with cycles
    Assert(Assembler::IsImmAddSub(cycles_to_add));
    armAsm->ldr(RWSCRATCH, PTR(&g_state.pending_ticks));
    armAsm->add(RWSCRATCH, RWSCRATCH, cycles_to_add);
    armAsm->str(RWSCRATCH, PTR(&g_state.pending_ticks));
  }

  if (address_register != static_cast<u8>(RWARG1.GetCode()))
    armAsm->mov(RWARG1, WRegister(address_register));

  if (!is_load)
  {
    if (address_register != static_cast<u8>(RWARG2.GetCode()))
      armAsm->mov(RWARG2, WRegister(data_register));
  }

  switch (size)
  {
    case MemoryAccessSize::Byte:
    {
      armEmitCall(armAsm,
                  is_load ? reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryByte) :
                            reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryByte),
                  false);
    }
    break;
    case MemoryAccessSize::HalfWord:
    {
      armEmitCall(armAsm,
                  is_load ? reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryHalfWord) :
                            reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryHalfWord),
                  false);
    }
    break;
    case MemoryAccessSize::Word:
    {
      armEmitCall(armAsm,
                  is_load ? reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedReadMemoryWord) :
                            reinterpret_cast<const void*>(&Recompiler::Thunks::UncheckedWriteMemoryWord),
                  false);
    }
    break;
  }

  if (is_load)
  {
    const WRegister dst = WRegister(data_register);
    switch (size)
    {
      case MemoryAccessSize::Byte:
      {
        is_signed ? armAsm->sxtb(dst, RWRET) : armAsm->uxtb(dst, RWRET);
      }
      break;
      case MemoryAccessSize::HalfWord:
      {
        is_signed ? armAsm->sxth(dst, RWRET) : armAsm->uxth(dst, RWRET);
      }
      break;
      case MemoryAccessSize::Word:
      {
        if (dst.GetCode() != RWRET.GetCode())
          armAsm->mov(dst, RWRET);
      }
      break;
    }
  }

  if (cycles_to_remove != 0)
  {
    Assert(Assembler::IsImmAddSub(cycles_to_add));
    armAsm->ldr(RWSCRATCH, PTR(&g_state.pending_ticks));
    armAsm->sub(RWSCRATCH, RWSCRATCH, cycles_to_add);
    armAsm->str(RWSCRATCH, PTR(&g_state.pending_ticks));
  }

  // restore regs
  if (stack_size > 0)
  {
    u32 stack_offset = 0;
    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      if ((gpr_bitmask & (1u << i)) && rvIsCallerSavedRegister(i) && (!is_load || data_register != i))
      {
        armAsm->ldr(XRegister(i), MemOperand(sp, stack_offset));
        stack_offset += GPR_SIZE;
      }
    }

    armAsm->add(sp, sp, stack_size);
  }

  armEmitJmp(armAsm, static_cast<const u8*>(code_address) + code_size, true);
  armAsm->FinalizeCode();

  const u32 thunk_size = static_cast<u32>(armAsm->GetCursorOffset());
  rvFlushInstructionCache(code_address, thunk_size);

  // backpatch to a jump to the slowmem handler
  EmitJump(code_address, armAsm->GetBuffer()->GetStartAddress<const void*>(), true);

#ifdef _DEBUG
  rvDisassembleAndDumpCode(thunk_code, thunk_size);
#endif

  return thunk_size;
}

#endif