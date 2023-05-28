// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "cpu_newrec_compiler.h"
#include "common/assert.h"
#include "common/log.h"
#include "common/string.h"
#include "cpu_code_cache.h"
#include "cpu_core_private.h"
#include "cpu_disasm.h"
#include "cpu_newrec_private.h"
#include "settings.h"
#include <cstdint>
#include <limits>
Log_SetChannel(NewRec::Compiler);

CPU::NewRec::Compiler::Compiler() = default;

CPU::NewRec::Compiler::~Compiler() = default;

void CPU::NewRec::Compiler::Reset(Block* block, u8* code_buffer, u32 code_buffer_space, u8* far_code_buffer,
                                  u32 far_code_space)
{
  m_block = block;
  m_compiler_pc = block->pc;
  m_cycles = 0;
  m_gte_done_cycle = 0;
  inst = nullptr;
  m_current_instruction_pc = 0;
  m_current_instruction_branch_delay_slot = false;
  m_dirty_pc = false;
  m_dirty_instruction_bits = false;
  m_dirty_gte_done_cycle = true;
  m_block_ended = false;
  m_constant_reg_values.fill(0);
  m_constant_regs_valid.reset();
  m_constant_regs_dirty.reset();

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
    ClearHostReg(i);
  m_register_alloc_counter = 0;

  m_constant_reg_values[static_cast<u32>(Reg::zero)] = 0;
  m_constant_regs_valid.set(static_cast<u32>(Reg::zero));

  m_load_delay_dirty = EMULATE_LOAD_DELAYS;
  m_load_delay_register = Reg::count;
  m_load_delay_value_register = NUM_HOST_REGS;
}

void CPU::NewRec::Compiler::BeginBlock()
{
  inst = m_block->Instructions();
  m_current_instruction_pc = m_block->pc;
  m_current_instruction_branch_delay_slot = false;
  m_compiler_pc += sizeof(Instruction);
  m_dirty_pc = true;
  m_dirty_instruction_bits = true;
}

const void* CPU::NewRec::Compiler::CompileBlock(Block* block)
{
  JitCodeBuffer& buffer = CodeCache::GetCodeBuffer();
  Reset(block, buffer.GetFreeCodePointer(), buffer.GetFreeCodeSpace(), buffer.GetFreeFarCodePointer(),
        buffer.GetFreeFarCodeSpace());

  Log_DebugPrintf("Block range: %08X -> %08X", block->pc, block->pc + block->size * 4);

  BeginBlock();

  while (!m_block_ended)
  {
    CompileInstruction();

    inst++;
    m_current_instruction_pc += sizeof(Instruction);
    m_compiler_pc += sizeof(Instruction);
    m_dirty_pc = true;
    m_dirty_instruction_bits = true;
  }

  // Nothing should be valid anymore
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
    DebugAssert(!IsHostRegAllocated(i));
  for (u32 i = 1; i < static_cast<u32>(Reg::count); i++)
    DebugAssert(!m_constant_regs_dirty.test(i) && !m_constant_regs_valid.test(i));

  u32 code_size, far_code_size;
  const void* code = EndCompile(&code_size, &far_code_size);
  buffer.CommitCode(code_size);
  buffer.CommitFarCode(far_code_size);

#ifdef _DEBUG
  const u32 host_instructions = GetHostInstructionCount(code, code_size);
  Log_ProfilePrintf("0x%08X: %u/%ub for %ub (%u inst), blowup: %.2fx, cache: %.2f%%/%.2f%%, ipi: %.2f", block->pc,
                    code_size, far_code_size, block->size * 4, block->size,
                    static_cast<float>(code_size) / static_cast<float>(block->size * 4),
                    buffer.GetUsedPct(), buffer.GetFarUsedPct(),
                    static_cast<float>(host_instructions) / static_cast<float>(block->size));
#else
  Log_ProfilePrintf("0x%08X: %u/%ub for %ub (%u inst), blowup: %.2fx, cache: %.2f%%/%.2f%%", block->pc, code_size,
                    far_code_size, block->size * 4, block->size,
                    static_cast<float>(code_size) / static_cast<float>(block->size * 4),
                    buffer.GetUsedPct(), buffer.GetFarUsedPct());
#endif

  return code;
}

void CPU::NewRec::Compiler::SetConstantReg(Reg r, u32 v)
{
  DebugAssert(r < Reg::count && r != Reg::zero);

  // There might still be an incoming load delay which we need to cancel.
  CancelLoadDelaysToReg(r);

  if (m_constant_regs_valid.test(static_cast<u32>(r)) && m_constant_reg_values[static_cast<u8>(r)] == v)
  {
    // Shouldn't be any host regs though.
    DebugAssert(!CheckHostReg(0, HR_TYPE_CPU_REG, r).has_value());
    return;
  }

  m_constant_reg_values[static_cast<u32>(r)] = v;
  m_constant_regs_valid.set(static_cast<u32>(r));
  m_constant_regs_dirty.set(static_cast<u32>(r));

  if (const std::optional<u32> hostreg = CheckHostReg(0, HR_TYPE_CPU_REG, r); hostreg.has_value())
  {
    Log_DebugPrintf("Discarding guest register %s in host register %s due to constant set", GetRegName(r),
                    GetHostRegName(hostreg.value()));
    FreeHostReg(hostreg.value());
  }
}

void CPU::NewRec::Compiler::CancelLoadDelaysToReg(Reg reg)
{
  if (m_load_delay_register != reg)
    return;

  Log_DebugPrintf("Cancelling load delay to %s", GetRegName(reg));
  m_load_delay_register = Reg::count;
  if (m_load_delay_value_register != NUM_HOST_REGS)
    ClearHostReg(m_load_delay_value_register);
}

void CPU::NewRec::Compiler::UpdateLoadDelay()
{
  if (m_load_delay_dirty)
  {
    // we shouldn't have a static load delay.
    DebugAssert(!HasLoadDelay());

    // have to invalidate registers, we might have one of them cached
    // TODO: double check the order here, will we trash a new value? we shouldn't...
    // thankfully since this only happens on the first instruction, we can get away with just killing anything which
    // isn't in write mode, because nothing could've been written before it, and the new value overwrites any
    // load-delayed value
    Log_DebugPrintf("Invalidating non-dirty registers, and flushing load delay from state");

    constexpr u32 req_flags = (HR_ALLOCATED | HR_MODE_WRITE);

    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      HostRegAlloc& ra = m_host_regs[i];
      if (ra.type != HR_TYPE_CPU_REG || !IsHostRegAllocated(i) || ((ra.flags & req_flags) == req_flags))
        continue;

      Log_DebugPrintf("Freeing non-dirty cached register %s in %s", GetRegName(ra.reg), GetHostRegName(i));
      DebugAssert(!(ra.flags & HR_MODE_WRITE));
      ClearHostReg(i);
    }

    // remove any non-dirty constants too
    for (u32 i = 1; i < static_cast<u32>(Reg::count); i++)
    {
      if (!HasConstantReg(static_cast<Reg>(i)) || HasDirtyConstantReg(static_cast<Reg>(i)))
        continue;

      Log_DebugPrintf("Clearing non-dirty constant %s", GetRegName(static_cast<Reg>(i)));
      ClearConstantReg(static_cast<Reg>(i));
    }

    Flush(FLUSH_LOAD_DELAY_FROM_STATE);
  }

  // commit the delayed register load
  FinishLoadDelay();

  // move next load delay forward
  if (m_next_load_delay_register != Reg::count)
  {
    // if it somehow got flushed, read it back in.
    if (m_next_load_delay_value_register == NUM_HOST_REGS)
    {
      AllocateHostReg(HR_MODE_READ, HR_TYPE_NEXT_LOAD_DELAY_VALUE, m_next_load_delay_register);
      DebugAssert(m_next_load_delay_value_register != NUM_HOST_REGS);
    }

    HostRegAlloc& ra = m_host_regs[m_next_load_delay_value_register];
    ra.flags |= HR_MODE_WRITE;
    ra.type = HR_TYPE_LOAD_DELAY_VALUE;

    m_load_delay_register = m_next_load_delay_register;
    m_load_delay_value_register = m_next_load_delay_value_register;
    m_next_load_delay_register = Reg::count;
    m_next_load_delay_value_register = NUM_HOST_REGS;
  }
}

void CPU::NewRec::Compiler::FinishLoadDelay()
{
  DebugAssert(!m_load_delay_dirty);
  if (!HasLoadDelay())
    return;

  // we may need to reload the value..
  if (m_load_delay_value_register == NUM_HOST_REGS)
  {
    AllocateHostReg(HR_MODE_READ, HR_TYPE_LOAD_DELAY_VALUE, m_load_delay_register);
    DebugAssert(m_load_delay_value_register != NUM_HOST_REGS);
  }

  // kill any (old) cached value for this register
  DeleteMIPSReg(m_load_delay_register, false);

  Log_DebugPrintf("Finished delayed load to %s in host register %s", GetRegName(m_load_delay_register),
                  GetHostRegName(m_load_delay_value_register));

  // and swap the mode over so it gets written back later
  HostRegAlloc& ra = m_host_regs[m_load_delay_value_register];
  DebugAssert(ra.reg == m_load_delay_register);
  ra.flags = (ra.flags & IMMUTABLE_HR_FLAGS) | HR_ALLOCATED | HR_MODE_READ | HR_MODE_WRITE;
  ra.counter = m_register_alloc_counter++;
  ra.type = HR_TYPE_CPU_REG;

  // constants are gone
  Log_DebugPrintf("Clearing constant in %s due to load delay", GetRegName(m_load_delay_register));
  ClearConstantReg(m_load_delay_register);

  m_load_delay_register = Reg::count;
  m_load_delay_value_register = NUM_HOST_REGS;
}

void CPU::NewRec::Compiler::FinishLoadDelayToReg(Reg reg)
{
  if (m_load_delay_dirty)
  {
    // inter-block :(
    UpdateLoadDelay();
    return;
  }

  if (m_load_delay_register != reg)
    return;

  FinishLoadDelay();
}

void CPU::NewRec::Compiler::ClearConstantReg(Reg r)
{
  DebugAssert(r < Reg::count && r != Reg::zero);
  m_constant_reg_values[static_cast<u32>(r)] = 0;
  m_constant_regs_valid.reset(static_cast<u32>(r));
  m_constant_regs_dirty.reset(static_cast<u32>(r));
}

void CPU::NewRec::Compiler::FlushConstantRegs(bool invalidate)
{
  for (u32 i = 1; i < static_cast<u32>(Reg::count); i++)
  {
    if (m_constant_regs_dirty.test(static_cast<u32>(i)))
      FlushConstantReg(static_cast<Reg>(i));
    if (invalidate)
      ClearConstantReg(static_cast<Reg>(i));
  }
}

CPU::Reg CPU::NewRec::Compiler::MipsD() const
{
  return inst->r.rd;
}

u32 CPU::NewRec::Compiler::GetConditionalBranchTarget(CompileFlags cf) const
{
  // compiler pc has already been advanced when swapping branch delay slots
  const u32 current_pc = m_compiler_pc - (cf.delay_slot_swapped ? sizeof(Instruction) : 0);
  return current_pc + (inst->i.imm_sext32() << 2);
}

u32 CPU::NewRec::Compiler::GetBranchReturnAddress(CompileFlags cf) const
{
  // compiler pc has already been advanced when swapping branch delay slots
  return m_compiler_pc + (cf.delay_slot_swapped ? 0 : sizeof(Instruction));
}

bool CPU::NewRec::Compiler::TrySwapDelaySlot(Reg rs, Reg rt, Reg rd)
{
  if constexpr (!SWAP_BRANCH_DELAY_SLOTS)
    return false;

  const Instruction* next_instruction = inst + 1;
  DebugAssert(next_instruction < (m_block->Instructions() + m_block->size));

  const Reg opcode_rs = next_instruction->r.rs;
  const Reg opcode_rt = next_instruction->r.rt;
  const Reg opcode_rd = next_instruction->r.rd;

#ifdef _DEBUG
  TinyString disasm;
  DisassembleInstruction(&disasm, m_current_instruction_pc + 4, next_instruction->bits);
#endif

  // Just in case we read it in the instruction.. but the block should end after this.
  const Instruction* const backup_instruction = inst;
  const u32 backup_instruction_pc = m_current_instruction_pc;
  const bool backup_instruction_delay_slot = m_current_instruction_branch_delay_slot;

  if (next_instruction->bits == 0)
  {
    // nop
    goto is_safe;
  }

  // can't swap when the branch is the first instruction because of bloody load delays
  if ((EMULATE_LOAD_DELAYS && m_block->pc == m_current_instruction_pc) || m_load_delay_dirty ||
      (HasLoadDelay() && (m_load_delay_register == rs || m_load_delay_register == rt || m_load_delay_register == rd)))
  {
    goto is_unsafe;
  }

  switch (next_instruction->op)
  {
    case InstructionOp::addi:
    case InstructionOp::addiu:
    case InstructionOp::slti:
    case InstructionOp::sltiu:
    case InstructionOp::andi:
    case InstructionOp::ori:
    case InstructionOp::xori:
    case InstructionOp::lui:
    case InstructionOp::lb:
    case InstructionOp::lh:
    case InstructionOp::lwl:
    case InstructionOp::lw:
    case InstructionOp::lbu:
    case InstructionOp::lhu:
    case InstructionOp::lwr:
    case InstructionOp::sb:
    case InstructionOp::sh:
    case InstructionOp::swl:
    case InstructionOp::sw:
    case InstructionOp::swr:
    {
      if ((rs != Reg::zero && rs == opcode_rt) || (rt != Reg::zero && rt == opcode_rt) ||
          (rd != Reg::zero && (rd == opcode_rs || rd == opcode_rt)) ||
          (HasLoadDelay() && (m_load_delay_register == opcode_rs || m_load_delay_register == opcode_rt)))
      {
        goto is_unsafe;
      }
    }
    break;

    case InstructionOp::lwc2: // LWC2
    case InstructionOp::swc2: // SWC2
      break;

    case InstructionOp::funct: // SPECIAL
    {
      switch (next_instruction->r.funct)
      {
        case InstructionFunct::sll:
        case InstructionFunct::srl:
        case InstructionFunct::sra:
        case InstructionFunct::sllv:
        case InstructionFunct::srlv:
        case InstructionFunct::srav:
        case InstructionFunct::add:
        case InstructionFunct::addu:
        case InstructionFunct::sub:
        case InstructionFunct::subu:
        case InstructionFunct::and_:
        case InstructionFunct::or_:
        case InstructionFunct::xor_:
        case InstructionFunct::nor:
        case InstructionFunct::slt:
        case InstructionFunct::sltu:
        {
          if ((rs != Reg::zero && rs == opcode_rd) || (rt != Reg::zero && rt == opcode_rd) ||
              (rd != Reg::zero && (rd == opcode_rs || rd == opcode_rt)) ||
              (HasLoadDelay() && (m_load_delay_register == opcode_rs || m_load_delay_register == opcode_rt ||
                                  m_load_delay_register == opcode_rd)))
          {
            goto is_unsafe;
          }
        }
        break;

        case InstructionFunct::mult:
        case InstructionFunct::multu:
        case InstructionFunct::div:
        case InstructionFunct::divu:
        {
          if (HasLoadDelay() && (m_load_delay_register == opcode_rs || m_load_delay_register == opcode_rt))
            goto is_unsafe;
        }
        break;

        default:
          goto is_unsafe;
      }
    }
    break;

    case InstructionOp::cop0: // COP0
    case InstructionOp::cop1: // COP1
    case InstructionOp::cop2: // COP2
    case InstructionOp::cop3: // COP3
    {
      if (next_instruction->cop.IsCommonInstruction())
      {
        switch (next_instruction->cop.CommonOp())
        {
          case CopCommonInstruction::mfcn: // MFC0
          case CopCommonInstruction::cfcn: // CFC0
          {
            if ((rs != Reg::zero && rs == opcode_rt) || (rt != Reg::zero && rt == opcode_rt) ||
                (rd != Reg::zero && rd == opcode_rt) || (HasLoadDelay() && m_load_delay_register == opcode_rt))
            {
              goto is_unsafe;
            }
          }
          break;

          case CopCommonInstruction::mtcn: // MTC0
          case CopCommonInstruction::ctcn: // CTC0
            break;
        }
      }
      else
      {
        // swap when it's GTE
        if (next_instruction->op != InstructionOp::cop2)
          goto is_unsafe;
      }
    }
    break;

    default:
      goto is_unsafe;
  }

is_safe:
#ifdef _DEBUG
  Log_DevPrintf("Swapping delay slot %08X %s", m_current_instruction_pc + 4, disasm.GetCharArray());
#endif

  CompileBranchDelaySlot();

  inst = backup_instruction;
  m_current_instruction_pc = backup_instruction_pc;
  m_current_instruction_branch_delay_slot = backup_instruction_delay_slot;
  return true;

is_unsafe:
#ifdef _DEBUG
  Log_DevPrintf("NOT swapping delay slot %08X %s", m_current_instruction_pc + 4, disasm.GetCharArray());
#endif

  return false;
}

void CPU::NewRec::Compiler::SetCompilerPC(u32 newpc)
{
  m_compiler_pc = newpc;
  m_dirty_pc = true;
}

std::optional<u32> CPU::NewRec::Compiler::GetFreeHostReg(u32 flags)
{
  const u32 req_flags = HR_USABLE | (flags & HR_CALLEE_SAVED);

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    if ((m_host_regs[i].flags & (req_flags | HR_ALLOCATED)) == req_flags)
      return i;
  }

  // find register with lowest counter
  // TODO: used analysis
  // TODO: when allocating a caller-saved reg and there's none free, swap a callee-saved reg
  u32 lowest = NUM_HOST_REGS;
  u16 lowest_count = std::numeric_limits<u16>::max();
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    const HostRegAlloc& ra = m_host_regs[i];
    if ((ra.flags & req_flags) != req_flags)
      continue;

    DebugAssert(ra.flags & HR_ALLOCATED);
    if (ra.type == HR_TYPE_TEMP)
    {
      // can't punt temps
      continue;
    }

    if (ra.counter < lowest_count)
    {
      lowest = i;
      lowest_count = ra.counter;
    }
  }

  if (lowest == NUM_HOST_REGS)
    return std::nullopt;

  const HostRegAlloc& ra = m_host_regs[lowest];
  switch (ra.type)
  {
    case HR_TYPE_CPU_REG:
    {
      Log_DebugPrintf("Freeing register %s in host register %s due for allocation", GetHostRegName(lowest),
                      GetRegName(ra.reg));
    }
    break;
    default:
    {
      Panic("Unknown type freed");
    }
    break;
  }

  FreeHostReg(lowest);
  return lowest;
}

const char* CPU::NewRec::Compiler::GetReadWriteModeString(u32 flags)
{
  if ((flags & (HR_MODE_READ | HR_MODE_WRITE)) == (HR_MODE_READ | HR_MODE_WRITE))
    return "read-write";
  else if (flags & HR_MODE_READ)
    return "read-only";
  else if (flags & HR_MODE_WRITE)
    return "write-only";
  else
    return "UNKNOWN";
}

u32 CPU::NewRec::Compiler::AllocateHostReg(u32 flags, HostRegAllocType type /* = HR_TYPE_TEMP */,
                                           Reg reg /* = Reg::count */)
{
  // Cancel any load delays before booting anything out
  if (flags & HR_MODE_WRITE && (type == HR_TYPE_CPU_REG || type == HR_TYPE_NEXT_LOAD_DELAY_VALUE))
    CancelLoadDelaysToReg(reg);

  // Already have a matching type?
  std::optional<u32> hreg;
  if (type != HR_TYPE_TEMP)
  {
    hreg = CheckHostReg(flags, type, reg);

    // shouldn't be allocating >1 load delay in a single instruction..
    // TODO: prefer callee saved registers for load delay
    DebugAssert((type != HR_TYPE_LOAD_DELAY_VALUE && type != HR_TYPE_NEXT_LOAD_DELAY_VALUE) || !hreg.has_value());
    if (hreg.has_value())
      return hreg.value();
  }

  hreg = GetFreeHostReg(flags);

  // TODO: kicking out when full
  if (!hreg.has_value())
    Panic("RA failed");

  HostRegAlloc& ra = m_host_regs[hreg.value()];
  ra.flags = (ra.flags & IMMUTABLE_HR_FLAGS) | (flags & ALLOWED_HR_FLAGS) | HR_ALLOCATED | HR_NEEDED;
  ra.type = type;
  ra.reg = reg;
  ra.counter = m_register_alloc_counter++;

  switch (type)
  {
    case HR_TYPE_CPU_REG:
    {
      DebugAssert(reg != Reg::zero);

      Log_DebugPrintf("Allocate host reg %s to guest reg %s in %s mode", GetHostRegName(hreg.value()), GetRegName(reg),
                      GetReadWriteModeString(flags));

      if (flags & HR_MODE_READ)
      {
        DebugAssert(ra.reg > Reg::zero && ra.reg < Reg::count);

        if (HasConstantReg(reg))
        {
          // may as well flush it now
          Log_DebugPrintf("Flush constant register in guest reg %s to host reg %s", GetRegName(reg),
                          GetHostRegName(hreg.value()));
          LoadHostRegWithConstant(hreg.value(), GetConstantRegU32(reg));
          m_constant_regs_dirty.reset(static_cast<u8>(reg));
          ra.flags |= HR_MODE_WRITE;
        }
        else
        {
          LoadHostRegFromCPUPointer(hreg.value(), &g_state.regs.r[static_cast<u8>(reg)]);
        }
      }

      if (flags & HR_MODE_WRITE && HasConstantReg(reg))
      {
        DebugAssert(reg != Reg::zero);
        Log_DebugPrintf("Clearing constant register in guest reg %s due to write mode in %s", GetRegName(reg),
                        GetHostRegName(hreg.value()));

        ClearConstantReg(reg);
      }
    }
    break;

    case HR_TYPE_LOAD_DELAY_VALUE:
    {
      DebugAssert(!m_load_delay_dirty && (!HasLoadDelay() || !(flags & HR_MODE_WRITE)));
      Log_DebugPrintf("Allocating load delayed guest register %s in host reg %s in %s mode", GetRegName(reg),
                      GetHostRegName(hreg.value()), GetReadWriteModeString(flags));
      m_load_delay_register = reg;
      m_load_delay_value_register = hreg.value();
      if (flags & HR_MODE_READ)
        LoadHostRegFromCPUPointer(hreg.value(), &g_state.load_delay_value);
    }
    break;

    case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
    {
      Log_DebugPrintf("Allocating next load delayed guest register %s in host reg %s in %s mode", GetRegName(reg),
                      GetHostRegName(hreg.value()), GetReadWriteModeString(flags));
      m_next_load_delay_register = reg;
      m_next_load_delay_value_register = hreg.value();
      if (flags & HR_MODE_READ)
        LoadHostRegFromCPUPointer(hreg.value(), &g_state.next_load_delay_value);
    }
    break;

    case HR_TYPE_TEMP:
    {
      DebugAssert(!(flags & (HR_MODE_READ | HR_MODE_WRITE)));
      Log_DebugPrintf("Allocate host reg %s as temporary", GetHostRegName(hreg.value()));
    }
    break;

    default:
      Panic("Unknown type");
      break;
  }

  return hreg.value();
}

std::optional<u32> CPU::NewRec::Compiler::CheckHostReg(u32 flags, HostRegAllocType type /* = HR_TYPE_TEMP */,
                                                       Reg reg /* = Reg::count */)
{
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (!(ra.flags & HR_ALLOCATED) || ra.type != type || ra.reg != reg)
      continue;

    DebugAssert(ra.flags & HR_MODE_READ);
    if (flags & HR_MODE_WRITE)
    {
      DebugAssert(type == HR_TYPE_CPU_REG);
      if (!(ra.flags & HR_MODE_WRITE))
      {
        Log_DebugPrintf("Switch guest reg %s from read to read-write in host reg %s", GetRegName(reg),
                        GetHostRegName(i));
      }

      if (HasConstantReg(reg))
      {
        DebugAssert(reg != Reg::zero);
        Log_DebugPrintf("Clearing constant register in guest reg %s due to write mode in %s", GetRegName(reg),
                        GetHostRegName(i));

        ClearConstantReg(reg);
      }
    }

    ra.flags |= (flags & ALLOWED_HR_FLAGS) | HR_NEEDED;
    ra.counter = m_register_alloc_counter++;

    // Need a callee saved reg?
    if (flags & HR_CALLEE_SAVED && !(ra.flags & HR_CALLEE_SAVED))
    {
      // Need to move it to one which is
      std::optional<u32> hreg = GetFreeHostReg(HR_CALLEE_SAVED);
      if (!hreg.has_value())
        Panic("RA failed");

      Log_DebugPrintf("Rename host reg %s to %s for callee saved", GetHostRegName(i), GetHostRegName(hreg.value()));

      std::swap(m_host_regs[i], m_host_regs[hreg.value()]);
      DebugAssert(!IsHostRegAllocated(i));
      i = hreg.value();
    }

    return i;
  }

  return std::nullopt;
}

u32 CPU::NewRec::Compiler::AllocateTempHostReg(u32 flags)
{
  return AllocateHostReg(flags, HR_TYPE_TEMP);
}

void CPU::NewRec::Compiler::FlushHostReg(u32 reg)
{
  HostRegAlloc& ra = m_host_regs[reg];
  if (ra.flags & HR_MODE_WRITE)
  {
    switch (ra.type)
    {
      case HR_TYPE_CPU_REG:
      {
        DebugAssert(ra.reg > Reg::zero && ra.reg < Reg::count);
        Log_DebugPrintf("Flushing register %s in host register %s to state", GetRegName(ra.reg), GetHostRegName(reg));
        StoreHostRegToCPUPointer(reg, &g_state.regs.r[static_cast<u8>(ra.reg)]);
      }
      break;

      case HR_TYPE_LOAD_DELAY_VALUE:
      {
        DebugAssert(m_load_delay_value_register == reg);
        Log_DebugPrintf("Flushing load delayed register %s in host register %s to state", GetRegName(ra.reg),
                        GetHostRegName(reg));

        StoreHostRegToCPUPointer(reg, &g_state.load_delay_value);
        m_load_delay_value_register = NUM_HOST_REGS;
      }
      break;

      case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
      {
        DebugAssert(m_next_load_delay_value_register == reg);
        Log_WarningPrintf("Flushing NEXT load delayed register %s in host register %s to state", GetRegName(ra.reg),
                          GetHostRegName(reg));

        StoreHostRegToCPUPointer(reg, &g_state.next_load_delay_value);
        m_next_load_delay_value_register = NUM_HOST_REGS;
      }
      break;
    }

    ra.flags = (ra.flags & ~HR_MODE_WRITE) | HR_MODE_READ;
  }
}

void CPU::NewRec::Compiler::FreeHostReg(u32 reg)
{
  DebugAssert(IsHostRegAllocated(reg));
  FlushHostReg(reg);
  ClearHostReg(reg);
}

void CPU::NewRec::Compiler::ClearHostReg(u32 reg)
{
  HostRegAlloc& ra = m_host_regs[reg];
  ra.flags &= IMMUTABLE_HR_FLAGS;
  ra.type = HR_TYPE_TEMP;
  ra.counter = 0;
  ra.reg = Reg::count;
}

void CPU::NewRec::Compiler::MarkRegsNeeded(HostRegAllocType type, Reg reg)
{
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (ra.flags & HR_ALLOCATED && ra.type == type && ra.reg == reg)
      ra.flags |= HR_NEEDED;
  }
}

void CPU::NewRec::Compiler::RenameHostReg(u32 reg, u32 new_flags, HostRegAllocType new_type, Reg new_reg)
{
  // only supported for cpu regs for now
  DebugAssert(new_type == HR_TYPE_TEMP || new_type == HR_TYPE_CPU_REG || new_type == HR_TYPE_NEXT_LOAD_DELAY_VALUE);

  const std::optional<u32> old_reg = CheckHostReg(0, new_type, new_reg);
  if (old_reg.has_value())
  {
    // don't writeback
    ClearHostReg(old_reg.value());
  }

  // kill any load delay to this reg
  if (new_type == HR_TYPE_CPU_REG || new_type == HR_TYPE_NEXT_LOAD_DELAY_VALUE)
    CancelLoadDelaysToReg(new_reg);

  if (new_type == HR_TYPE_CPU_REG)
  {
    Log_DebugPrintf("Renaming host reg %s to guest reg %s", GetHostRegName(reg), GetRegName(new_reg));
  }
  else if (new_type == HR_TYPE_NEXT_LOAD_DELAY_VALUE)
  {
    Log_DebugPrintf("Renaming host reg %s to load delayed guest reg %s", GetHostRegName(reg), GetRegName(new_reg));
    DebugAssert(m_next_load_delay_register == Reg::count && m_next_load_delay_value_register == NUM_HOST_REGS);
    m_next_load_delay_register = new_reg;
    m_next_load_delay_value_register = reg;
  }
  else
  {
    Log_DebugPrintf("Renaming host reg %s to temp", GetHostRegName(reg));
  }

  HostRegAlloc& ra = m_host_regs[reg];
  ra.flags = (ra.flags & IMMUTABLE_HR_FLAGS) | HR_NEEDED | HR_ALLOCATED | (new_flags & ALLOWED_HR_FLAGS);
  ra.counter = m_register_alloc_counter++;
  ra.type = new_type;
  ra.reg = new_reg;
}

void CPU::NewRec::Compiler::ClearHostRegNeeded(u32 reg)
{
  DebugAssert(reg < NUM_HOST_REGS && IsHostRegAllocated(reg));
  HostRegAlloc& ra = m_host_regs[reg];
  if (ra.flags & HR_MODE_WRITE)
    ra.flags |= HR_MODE_READ;

  ra.flags &= ~HR_NEEDED;
}

void CPU::NewRec::Compiler::ClearHostRegsNeeded()
{
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (!(ra.flags & HR_ALLOCATED))
      continue;

    // shouldn't have any temps left
    DebugAssert(ra.type != HR_TYPE_TEMP);

    if (ra.flags & HR_MODE_WRITE)
      ra.flags |= HR_MODE_READ;

    ra.flags &= ~HR_NEEDED;
  }
}

void CPU::NewRec::Compiler::DeleteMIPSReg(Reg reg, bool flush)
{
  DebugAssert(reg != Reg::zero);

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (ra.flags & HR_ALLOCATED && ra.type == HR_TYPE_CPU_REG && ra.reg == reg)
    {
      if (flush)
        FlushHostReg(i);
      ClearHostReg(i);
      ClearConstantReg(reg);
      return;
    }
  }

  if (flush)
    FlushConstantReg(reg);
  ClearConstantReg(reg);
}

void CPU::NewRec::Compiler::Flush(u32 flags)
{
  // TODO: Flush unneeded caller-saved regs (backup/replace calle-saved needed with caller-saved)
  if (flags &
      (FLUSH_FREE_UNNEEDED_CALLER_SAVED_REGISTERS | FLUSH_FREE_CALLER_SAVED_REGISTERS | FLUSH_FREE_ALL_REGISTERS))
  {
    const u32 req_mask = (flags & FLUSH_FREE_ALL_REGISTERS) ?
                           HR_ALLOCATED :
                           ((flags & FLUSH_FREE_CALLER_SAVED_REGISTERS) ? (HR_ALLOCATED | HR_CALLEE_SAVED) :
                                                                          (HR_ALLOCATED | HR_CALLEE_SAVED | HR_NEEDED));
    constexpr u32 req_flags = HR_ALLOCATED;

    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      HostRegAlloc& ra = m_host_regs[i];
      if ((ra.flags & req_mask) == req_flags)
        FreeHostReg(i);
    }
  }

  if (flags & FLUSH_INVALIDATE_MIPS_REGISTERS)
  {
    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      HostRegAlloc& ra = m_host_regs[i];
      if (ra.flags & HR_ALLOCATED && ra.type == HR_TYPE_CPU_REG)
        FreeHostReg(i);
    }

    FlushConstantRegs(true);
  }
  else
  {
    if (flags & FLUSH_FLUSH_MIPS_REGISTERS)
    {
      for (u32 i = 0; i < NUM_HOST_REGS; i++)
      {
        HostRegAlloc& ra = m_host_regs[i];
        if ((ra.flags & (HR_ALLOCATED | HR_MODE_WRITE)) == (HR_ALLOCATED | HR_MODE_WRITE) && ra.type == HR_TYPE_CPU_REG)
          FlushHostReg(i);
      }

      // flush any constant registers which are dirty too
      FlushConstantRegs(false);
    }
  }
}

void CPU::NewRec::Compiler::FlushConstantReg(Reg r)
{
  DebugAssert(m_constant_regs_valid.test(static_cast<u32>(r)));
  Log_DebugPrintf("Writing back register %s with constant value 0x%08X", GetRegName(r),
                  m_constant_reg_values[static_cast<u32>(r)]);
  StoreConstantToCPUPointer(m_constant_reg_values[static_cast<u32>(r)], &g_state.regs.r[static_cast<u32>(r)]);
  m_constant_regs_dirty.reset(static_cast<u32>(r));
}

void CPU::NewRec::Compiler::BackupHostState()
{
  DebugAssert(m_host_state_backup_count < m_host_state_backup.size());

  // need to back up everything...
  HostStateBackup& bu = m_host_state_backup[m_host_state_backup_count];
  bu.cycles = m_cycles;
  bu.gte_done_cycle = m_gte_done_cycle;
  bu.compiler_pc = m_compiler_pc;
  bu.dirty_pc = m_dirty_pc;
  bu.dirty_instruction_bits = m_dirty_instruction_bits;
  bu.dirty_gte_done_cycle = m_dirty_gte_done_cycle;
  bu.block_ended = m_block_ended;
  bu.inst = inst;
  bu.current_instruction_pc = m_current_instruction_pc;
  bu.current_instruction_delay_slot = m_current_instruction_branch_delay_slot;
  bu.const_regs_valid = m_constant_regs_valid;
  bu.const_regs_dirty = m_constant_regs_dirty;
  bu.const_regs_values = m_constant_reg_values;
  bu.host_regs = m_host_regs;
  bu.register_alloc_counter = m_register_alloc_counter;
  bu.load_delay_dirty = m_load_delay_dirty;
  bu.load_delay_register = m_load_delay_register;
  bu.load_delay_value_register = m_load_delay_value_register;
  bu.next_load_delay_register = m_next_load_delay_register;
  bu.next_load_delay_value_register = m_next_load_delay_value_register;
  m_host_state_backup_count++;
}

void CPU::NewRec::Compiler::RestoreHostState()
{
  DebugAssert(m_host_state_backup_count > 0);
  m_host_state_backup_count--;

  HostStateBackup& bu = m_host_state_backup[m_host_state_backup_count];
  m_host_regs = std::move(bu.host_regs);
  m_constant_reg_values = std::move(bu.const_regs_values);
  m_constant_regs_dirty = std::move(bu.const_regs_dirty);
  m_constant_regs_valid = std::move(bu.const_regs_valid);
  m_current_instruction_branch_delay_slot = bu.current_instruction_delay_slot;
  m_current_instruction_pc = bu.current_instruction_pc;
  inst = bu.inst;
  m_block_ended = bu.block_ended;
  m_dirty_gte_done_cycle = bu.dirty_gte_done_cycle;
  m_dirty_instruction_bits = bu.dirty_instruction_bits;
  m_dirty_pc = bu.dirty_pc;
  m_compiler_pc = bu.compiler_pc;
  m_register_alloc_counter = bu.register_alloc_counter;
  m_load_delay_dirty = bu.load_delay_dirty;
  m_load_delay_register = bu.load_delay_register;
  m_load_delay_value_register = bu.load_delay_value_register;
  m_next_load_delay_register = bu.next_load_delay_register;
  m_next_load_delay_value_register = bu.next_load_delay_value_register;
  m_gte_done_cycle = bu.gte_done_cycle;
  m_cycles = bu.cycles;
}

void CPU::NewRec::Compiler::AddLoadStoreInfo(void* code_address, u32 code_size, u32 address_register, u32 data_register,
                                             MemoryAccessSize size, bool is_signed, bool is_load)
{
  DebugAssert(g_settings.IsUsingFastmem());
  DebugAssert(address_register < NUM_HOST_REGS);
  DebugAssert(data_register < NUM_HOST_REGS);

  u32 gpr_bitmask = 0;
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    if (IsHostRegAllocated(i))
      gpr_bitmask |= (1u << i);
  }

  CPU::NewRec::AddLoadStoreInfo(code_address, code_size, m_current_instruction_pc, m_cycles, gpr_bitmask,
                                static_cast<u8>(address_register), static_cast<u8>(data_register), size, is_signed,
                                is_load);
}

void CPU::NewRec::Compiler::CompileInstruction()
{
#ifdef _DEBUG
  const void* start = GetCurrentCodePointer();

  TinyString str;
  DisassembleInstruction(&str, m_current_instruction_pc, inst->bits);
  Log_DebugPrintf("Compiling%s %08X: %s", m_current_instruction_branch_delay_slot ? " branch delay slot" : "",
                  m_current_instruction_pc, str.GetCharArray());
#endif

  m_cycles++;

  if (IsNopInstruction(*inst))
  {
    UpdateLoadDelay();
    return;
  }

  switch (inst->op)
  {
      // clang-format off

    case InstructionOp::funct:
    {
      switch (inst->r.funct)
      {
        case InstructionFunct::sll: CompileTemplate(&Compiler::Compile_sll_const, &Compiler::Compile_sll, TF_WRITES_D | TF_READS_T); break;
        case InstructionFunct::srl: CompileTemplate(&Compiler::Compile_srl_const, &Compiler::Compile_srl, TF_WRITES_D | TF_READS_T); break;
        case InstructionFunct::sra: CompileTemplate(&Compiler::Compile_sra_const, &Compiler::Compile_sra, TF_WRITES_D | TF_READS_T); break;
        case InstructionFunct::sllv: CompileTemplate(&Compiler::Compile_sllv_const, &Compiler::Compile_sllv, TF_WRITES_D | TF_READS_S | TF_READS_T); break;
        case InstructionFunct::srlv: CompileTemplate(&Compiler::Compile_srlv_const, &Compiler::Compile_srlv, TF_WRITES_D | TF_READS_S | TF_READS_T); break;
        case InstructionFunct::srav: CompileTemplate(&Compiler::Compile_srav_const, &Compiler::Compile_srav, TF_WRITES_D | TF_READS_S | TF_READS_T); break;
        case InstructionFunct::jr: CompileTemplate(&Compiler::Compile_jr_const, &Compiler::Compile_jr, TF_READS_S); break;
        case InstructionFunct::jalr: CompileTemplate(&Compiler::Compile_jalr_const, &Compiler::Compile_jalr, /*TF_WRITES_D |*/ TF_READS_S | TF_NO_NOP); break;
        case InstructionFunct::syscall: Compile_syscall(); break;
        case InstructionFunct::break_: Compile_break(); break;
        case InstructionFunct::mfhi: CompileMoveRegTemplate(inst->r.rd, Reg::hi); break;
        case InstructionFunct::mthi: CompileMoveRegTemplate(Reg::hi, inst->r.rs); break;
        case InstructionFunct::mflo: CompileMoveRegTemplate(inst->r.rd, Reg::lo); break;
        case InstructionFunct::mtlo: CompileMoveRegTemplate(Reg::lo, inst->r.rs); break;
        case InstructionFunct::mult: CompileTemplate(&Compiler::Compile_mult_const, &Compiler::Compile_mult, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI | TF_COMMUTATIVE); break;
        case InstructionFunct::multu: CompileTemplate(&Compiler::Compile_multu_const, &Compiler::Compile_multu, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI | TF_COMMUTATIVE); break;
        case InstructionFunct::div: CompileTemplate(&Compiler::Compile_div_const, &Compiler::Compile_div, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI); break;
        case InstructionFunct::divu: CompileTemplate(&Compiler::Compile_divu_const, &Compiler::Compile_divu, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI); break;
        case InstructionFunct::add: CompileTemplate(&Compiler::Compile_add_const, &Compiler::Compile_add, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE | TF_CAN_OVERFLOW); break;
        case InstructionFunct::addu: CompileTemplate(&Compiler::Compile_addu_const, &Compiler::Compile_addu, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
        case InstructionFunct::sub: CompileTemplate(&Compiler::Compile_sub_const, &Compiler::Compile_sub, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_CAN_OVERFLOW); break;
        case InstructionFunct::subu: CompileTemplate(&Compiler::Compile_subu_const, &Compiler::Compile_subu, TF_WRITES_D | TF_READS_S | TF_READS_T); break;
        case InstructionFunct::and_: CompileTemplate(&Compiler::Compile_and_const, &Compiler::Compile_and, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
        case InstructionFunct::or_: CompileTemplate(&Compiler::Compile_or_const, &Compiler::Compile_or, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
        case InstructionFunct::xor_: CompileTemplate(&Compiler::Compile_xor_const, &Compiler::Compile_xor, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
        case InstructionFunct::nor: CompileTemplate(&Compiler::Compile_nor_const, &Compiler::Compile_nor, TF_WRITES_D | TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
        case InstructionFunct::slt: CompileTemplate(&Compiler::Compile_slt_const, &Compiler::Compile_slt, TF_WRITES_D | TF_READS_T | TF_READS_S); break;
        case InstructionFunct::sltu: CompileTemplate(&Compiler::Compile_sltu_const, &Compiler::Compile_sltu, TF_WRITES_D | TF_READS_T | TF_READS_S); break;

      default: Panic("fixme funct"); break;
      }
    }
    break;

    case InstructionOp::j: Compile_j(); break;
    case InstructionOp::jal: Compile_jal(); break;

    case InstructionOp::b: CompileTemplate(&Compiler::Compile_b_const, &Compiler::Compile_b, TF_READS_S | TF_CAN_SWAP_DELAY_SLOT); break;
    case InstructionOp::blez: CompileTemplate(&Compiler::Compile_blez_const, &Compiler::Compile_blez, TF_READS_S | TF_CAN_SWAP_DELAY_SLOT); break;
    case InstructionOp::bgtz: CompileTemplate(&Compiler::Compile_bgtz_const, &Compiler::Compile_bgtz, TF_READS_S | TF_CAN_SWAP_DELAY_SLOT); break;
    case InstructionOp::beq: CompileTemplate(&Compiler::Compile_beq_const, &Compiler::Compile_beq, TF_READS_S | TF_READS_T | TF_COMMUTATIVE | TF_CAN_SWAP_DELAY_SLOT); break;
    case InstructionOp::bne: CompileTemplate(&Compiler::Compile_bne_const, &Compiler::Compile_bne, TF_READS_S | TF_READS_T | TF_COMMUTATIVE | TF_CAN_SWAP_DELAY_SLOT); break;

    case InstructionOp::addi: CompileTemplate(&Compiler::Compile_addi_const, &Compiler::Compile_addi, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE | TF_CAN_OVERFLOW); break;
    case InstructionOp::addiu: CompileTemplate(&Compiler::Compile_addiu_const, &Compiler::Compile_addiu, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::slti: CompileTemplate(&Compiler::Compile_slti_const, &Compiler::Compile_slti, TF_WRITES_T | TF_READS_S); break;
    case InstructionOp::sltiu: CompileTemplate(&Compiler::Compile_sltiu_const, &Compiler::Compile_sltiu, TF_WRITES_T | TF_READS_S); break;
    case InstructionOp::andi: CompileTemplate(&Compiler::Compile_andi_const, &Compiler::Compile_andi, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::ori: CompileTemplate(&Compiler::Compile_ori_const, &Compiler::Compile_ori, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::xori: CompileTemplate(&Compiler::Compile_xori_const, &Compiler::Compile_xori, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::lui: Compile_lui(); break;

    case InstructionOp::lb: CompileLoadStoreTemplate(&Compiler::Compile_lxx, MemoryAccessSize::Byte, false, true, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lbu: CompileLoadStoreTemplate(&Compiler::Compile_lxx, MemoryAccessSize::Byte, false, false, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lh: CompileLoadStoreTemplate(&Compiler::Compile_lxx, MemoryAccessSize::HalfWord, false, true, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lhu: CompileLoadStoreTemplate(&Compiler::Compile_lxx, MemoryAccessSize::HalfWord, false, false, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lw: CompileLoadStoreTemplate(&Compiler::Compile_lxx, MemoryAccessSize::Word, false, false, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lwl: CompileLoadStoreTemplate(&Compiler::Compile_lwx, MemoryAccessSize::Word, false, false, TF_READS_S | /*TF_READS_T | TF_WRITES_T | */TF_LOAD_DELAY); break;
    case InstructionOp::lwr: CompileLoadStoreTemplate(&Compiler::Compile_lwx, MemoryAccessSize::Word, false, false, TF_READS_S | /*TF_READS_T | TF_WRITES_T | */TF_LOAD_DELAY); break;
    case InstructionOp::sb: CompileLoadStoreTemplate(&Compiler::Compile_sxx, MemoryAccessSize::Byte, true, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::sh: CompileLoadStoreTemplate(&Compiler::Compile_sxx, MemoryAccessSize::HalfWord, true, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::sw: CompileLoadStoreTemplate(&Compiler::Compile_sxx, MemoryAccessSize::Word, true, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::swl: CompileLoadStoreTemplate(&Compiler::Compile_swx, MemoryAccessSize::Word, false, false, TF_READS_S | /*TF_READS_T | TF_WRITES_T | */TF_LOAD_DELAY); break;
    case InstructionOp::swr: CompileLoadStoreTemplate(&Compiler::Compile_swx, MemoryAccessSize::Word, false, false, TF_READS_S | /*TF_READS_T | TF_WRITES_T | */TF_LOAD_DELAY); break;

    case InstructionOp::cop0:
      {
        if (inst->cop.IsCommonInstruction())
        {
          switch (inst->cop.CommonOp())
          {
            case CopCommonInstruction::mfcn: CompileTemplate(nullptr, &Compiler::Compile_mfc0, TF_WRITES_T | TF_LOAD_DELAY); break;
            case CopCommonInstruction::mtcn: CompileTemplate(nullptr, &Compiler::Compile_mtc0, TF_READS_T); break;
            default: Compile_Fallback(); break;
          }
        }
        else
        {
          switch (inst->cop.Cop0Op())
          {
            case Cop0Instruction::rfe: CompileTemplate(nullptr, &Compiler::Compile_rfe, 0); break;
            default: Compile_Fallback(); break;
          }
        }
      }
      break;

    case InstructionOp::cop2:
      {
        if (inst->cop.IsCommonInstruction())
        {
          switch (inst->cop.CommonOp())
          {
            case CopCommonInstruction::mfcn: CompileTemplate(nullptr, &Compiler::Compile_mfc2, TF_GTE_STALL); break;
            case CopCommonInstruction::cfcn: CompileTemplate(nullptr, &Compiler::Compile_mfc2, TF_GTE_STALL); break;
            case CopCommonInstruction::mtcn: CompileTemplate(nullptr, &Compiler::Compile_mtc2, TF_GTE_STALL | TF_READS_T); break;
            case CopCommonInstruction::ctcn: CompileTemplate(nullptr, &Compiler::Compile_mtc2, TF_GTE_STALL | TF_READS_T); break;
            default: Compile_Fallback(); break;
          }
        }
        else
        {
          // GTE ops
          CompileTemplate(nullptr, &Compiler::Compile_cop2, TF_GTE_STALL);
        }
      }
      break;

    case InstructionOp::lwc2: CompileLoadStoreTemplate(&Compiler::Compile_lwc2, MemoryAccessSize::Word, false, false, TF_GTE_STALL | TF_READS_S | TF_LOAD_DELAY); break;
    case InstructionOp::swc2: CompileLoadStoreTemplate(&Compiler::Compile_swc2, MemoryAccessSize::Word, true, false, TF_GTE_STALL | TF_READS_S); break;

    default: Panic("Fixme"); break;
      // clang-format on
  }

  ClearHostRegsNeeded();
  UpdateLoadDelay();

#ifdef _DEBUG
  const void* end = GetCurrentCodePointer();
  if (start != end && !m_current_instruction_branch_delay_slot)
    DisassembleAndLog(start, static_cast<u32>(static_cast<const u8*>(end) - static_cast<const u8*>(start)));
#endif
}

void CPU::NewRec::Compiler::CompileBranchDelaySlot(bool dirty_pc /* = true */)
{
  // Update load delay at the end of the previous instruction.
  UpdateLoadDelay();

  // TODO: Move cycle add before this.
  inst++;
  m_current_instruction_pc += sizeof(Instruction);
  m_current_instruction_branch_delay_slot = true;
  m_compiler_pc += sizeof(Instruction);
  m_dirty_pc = dirty_pc;
  m_dirty_instruction_bits = true;

  CompileInstruction();

  m_current_instruction_branch_delay_slot = false;
}

void CPU::NewRec::Compiler::CompileTemplate(void (Compiler::*const_func)(CompileFlags),
                                            void (Compiler::*func)(CompileFlags), u32 tflags)
{
  // TODO: This is where we will do memory operand optimization. Remember to kill constants!
  // TODO: Swap S and T if commutative
  // TODO: For and, treat as zeroing if imm is zero
  // TODO: Optimize slt + bne to cmp + jump
  // TODO: Rename S to D for sll 0, addi 0, etc.
  // TODO: Prefer memory operands when load delay is dirty, since we're going to invalidate immediately after the first
  // instruction..
  // TODO: andi with zero -> zero const
  // TODO: load constant so it can be flushed if it's not overwritten later
  // TODO: inline PGXP ops.

  bool allow_constant = static_cast<bool>(const_func);
  Reg rs = inst->r.rs.GetValue();
  Reg rt = inst->r.rt.GetValue();
  Reg rd = inst->r.rd.GetValue();

  if (tflags & TF_GTE_STALL)
    StallUntilGTEComplete();

  // throw away instructions writing to $zero
  if (!(tflags & TF_NO_NOP) && (!g_settings.cpu_recompiler_memory_exceptions || !(tflags & TF_CAN_OVERFLOW)) &&
      ((tflags & TF_WRITES_T && rt == Reg::zero) || (tflags & TF_WRITES_D && rd == Reg::zero)))
  {
    Log_DebugPrintf("Skipping instruction because it writes to zero");
    return;
  }

  // if it's a commutative op, and we have one constant reg but not the other, swap them
  // TODO: make it swap when writing to T as well
  // TODO: drop the hack for rd == rt
  if (tflags & TF_COMMUTATIVE && !(tflags & TF_WRITES_T) &&
      ((HasConstantReg(rs) && !HasConstantReg(rt)) || (tflags & TF_WRITES_D && rd == rt)))
  {
    Log_DebugPrintf("Swapping S:%s and T:%s due to commutative op and constants", GetRegName(rs), GetRegName(rt));
    std::swap(rs, rt);
  }

  CompileFlags cf = {};

  if (tflags & TF_READS_S)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, rs);
    if (HasConstantReg(rs))
      cf.const_s = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_T)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, rt);
    if (HasConstantReg(rt))
      cf.const_t = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_LO)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, Reg::lo);
    if (HasConstantReg(Reg::lo))
      cf.const_lo = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_HI)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, Reg::hi);
    if (HasConstantReg(Reg::hi))
      cf.const_hi = true;
    else
      allow_constant = false;
  }

  // Needed because of potential swapping
  if (tflags & TF_READS_S)
    cf.mips_s = static_cast<u8>(rs);
  if (tflags & (TF_READS_T | TF_WRITES_T))
    cf.mips_t = static_cast<u8>(rt);

  if (allow_constant)
  {
    // woot, constant path
    (this->*const_func)(cf);
    return;
  }

  if (tflags & TF_CAN_SWAP_DELAY_SLOT && TrySwapDelaySlot(cf.MipsS(), cf.MipsT()))
    cf.delay_slot_swapped = true;

  if (tflags & TF_READS_S &&
      (tflags & TF_NEEDS_REG_S || !cf.const_s || (tflags & TF_WRITES_D && rd != Reg::zero && rd == rs)))
  {
    cf.host_s = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rs);
    cf.const_s = false;
    cf.valid_host_s = true;
  }

  if (tflags & TF_READS_T &&
      (tflags & (TF_NEEDS_REG_T | TF_WRITES_T) || !cf.const_t || (tflags & TF_WRITES_D && rd != Reg::zero && rd == rt)))
  {
    cf.host_t = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt);
    cf.const_t = false;
    cf.valid_host_t = true;
  }

  if (tflags & (TF_READS_LO | TF_WRITES_LO))
  {
    cf.host_lo =
      AllocateHostReg(((tflags & TF_READS_LO) ? HR_MODE_READ : 0u) | ((tflags & TF_WRITES_LO) ? HR_MODE_WRITE : 0u),
                      HR_TYPE_CPU_REG, Reg::lo);
    cf.const_lo = false;
    cf.valid_host_lo = true;
  }

  if (tflags & (TF_READS_HI | TF_WRITES_HI))
  {
    cf.host_hi =
      AllocateHostReg(((tflags & TF_READS_HI) ? HR_MODE_READ : 0u) | ((tflags & TF_WRITES_HI) ? HR_MODE_WRITE : 0u),
                      HR_TYPE_CPU_REG, Reg::hi);
    cf.const_hi = false;
    cf.valid_host_hi = true;
  }

  const HostRegAllocType write_type =
    (tflags & TF_LOAD_DELAY && EMULATE_LOAD_DELAYS) ? HR_TYPE_NEXT_LOAD_DELAY_VALUE : HR_TYPE_CPU_REG;

  if (tflags & TF_CAN_OVERFLOW && g_settings.cpu_recompiler_memory_exceptions)
  {
    // allocate a temp register for the result, then swap it back
    const u32 tempreg = AllocateHostReg(0, HR_TYPE_TEMP);
    ;
    if (tflags & TF_WRITES_D)
    {
      cf.host_d = tempreg;
      cf.valid_host_d = true;
    }
    else if (tflags & TF_WRITES_T)
    {
      cf.host_t = tempreg;
      cf.valid_host_t = true;
    }

    (this->*func)(cf);

    if (tflags & TF_WRITES_D && rd != Reg::zero)
    {
      DeleteMIPSReg(rd, false);
      RenameHostReg(tempreg, HR_MODE_WRITE, write_type, rd);
    }
    else if (tflags & TF_WRITES_T && rt != Reg::zero)
    {
      DeleteMIPSReg(rt, false);
      RenameHostReg(tempreg, HR_MODE_WRITE, write_type, rt);
    }
    else
    {
      FreeHostReg(tempreg);
    }
  }
  else
  {
    if (tflags & TF_WRITES_D && rd != Reg::zero)
    {
      cf.host_d = AllocateHostReg(HR_MODE_WRITE, write_type, rd);
      cf.valid_host_d = true;
    }

    if (tflags & TF_WRITES_T && rt != Reg::zero)
    {
      cf.host_t = AllocateHostReg(HR_MODE_WRITE, write_type, rt);
      cf.valid_host_t = true;
    }

    (this->*func)(cf);
  }
}

void CPU::NewRec::Compiler::CompileLoadStoreTemplate(void (Compiler::*func)(CompileFlags, MemoryAccessSize, bool,
                                                                            const std::optional<VirtualMemoryAddress>&),
                                                     MemoryAccessSize size, bool store, bool sign, u32 tflags)
{
  const Reg rs = inst->i.rs;
  const Reg rt = inst->i.rt;

  if (tflags & TF_GTE_STALL)
    StallUntilGTEComplete();

  CompileFlags cf = {};

  if (tflags & TF_READS_S)
    cf.mips_s = static_cast<u8>(rs);
  if (tflags & (TF_READS_T | TF_WRITES_T))
    cf.mips_t = static_cast<u8>(rt);

  // constant address?
  // TODO: move this to the backend address calculation?
  std::optional<VirtualMemoryAddress> addr;
  if (HasConstantReg(rs))
  {
    addr = GetConstantRegU32(rs) + inst->i.imm_sext32();
    cf.const_s = true;
  }
  else
  {
    if constexpr (HAS_MEMORY_OPERANDS)
    {
      // don't bother caching it since we're going to flush anyway
      // TODO: make less rubbish, if it's caller saved we don't need to flush...
      const std::optional<u32> hreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rs);
      if (hreg.has_value())
      {
        cf.valid_host_s = true;
        cf.host_s = hreg.value();
      }
    }
    else
    {
      // need rs in a register
      cf.host_s = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rs);
      cf.valid_host_s = true;
    }
  }

  // reads T -> store, writes T -> load
  // for now, we defer the allocation to afterwards, because C call
  if (tflags & TF_READS_T)
  {
    if (HasConstantReg(rt))
    {
      cf.const_t = true;
    }
    else
    {
      if constexpr (HAS_MEMORY_OPERANDS)
      {
        const std::optional<u32> hreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt);
        if (hreg.has_value())
        {
          cf.valid_host_t = true;
          cf.host_t = hreg.value();
        }
      }
      else
      {
        cf.host_t = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, rt);
        cf.valid_host_t = true;
      }
    }
  }

  (this->*func)(cf, size, sign, addr);
}

void CPU::NewRec::Compiler::CompileMoveRegTemplate(Reg dst, Reg src)
{
  if (dst == src || dst == Reg::zero)
    return;

  if (HasConstantReg(src))
  {
    DeleteMIPSReg(dst, false);
    SetConstantReg(dst, GetConstantRegU32(src));
    return;
  }

  // TODO: rename if src is no longer used
  const u32 srcreg = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, src);
  const u32 dstreg = AllocateHostReg(HR_MODE_WRITE, HR_TYPE_CPU_REG, dst);
  CopyHostReg(dstreg, srcreg);
}

void CPU::NewRec::Compiler::Compile_j()
{
  const u32 newpc = (m_compiler_pc & UINT32_C(0xF0000000)) | (inst->j.target << 2);

  // TODO: Delay slot swap.
  // We could also move the cycle commit back.
  CompileBranchDelaySlot();
  EndBlock(newpc);
}

void CPU::NewRec::Compiler::Compile_jr_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  const u32 newpc = GetConstantRegU32(cf.MipsS());
  if (newpc & 3 && g_settings.cpu_recompiler_memory_exceptions)
  {
    EndBlockWithException(Exception::AdEL);
    return;
  }

  CompileBranchDelaySlot();
  EndBlock(newpc);
}

void CPU::NewRec::Compiler::Compile_jal()
{
  const u32 newpc = (m_compiler_pc & UINT32_C(0xF0000000)) | (inst->j.target << 2);
  SetConstantReg(Reg::ra, GetBranchReturnAddress({}));
  CompileBranchDelaySlot();
  EndBlock(newpc);
}

void CPU::NewRec::Compiler::Compile_jalr_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  const u32 newpc = GetConstantRegU32(cf.MipsS());
  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetBranchReturnAddress({}));

  CompileBranchDelaySlot();
  EndBlock(newpc);
}

void CPU::NewRec::Compiler::Compile_syscall()
{
  EndBlockWithException(Exception::Syscall);
}

void CPU::NewRec::Compiler::Compile_break()
{
  EndBlockWithException(Exception::BP);
}

void CPU::NewRec::Compiler::Compile_b_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));

  const u8 irt = static_cast<u8>(inst->i.rt.GetValue());
  const bool bgez = ConvertToBoolUnchecked(irt & u8(1));
  const bool link = (irt & u8(0x1E)) == u8(0x10);

  const s32 rs = GetConstantRegS32(cf.MipsS());
  const bool taken = bgez ? (rs >= 0) : (rs < 0);
  const u32 taken_pc = GetConditionalBranchTarget(cf);

  if (link)
    SetConstantReg(Reg::ra, GetBranchReturnAddress(cf));

  CompileBranchDelaySlot();
  EndBlock(taken ? taken_pc : m_compiler_pc);
}

void CPU::NewRec::Compiler::Compile_b(CompileFlags cf)
{
  const u8 irt = static_cast<u8>(inst->i.rt.GetValue());
  const bool bgez = ConvertToBoolUnchecked(irt & u8(1));
  const bool link = (irt & u8(0x1E)) == u8(0x10);

  if (link)
    SetConstantReg(Reg::ra, GetBranchReturnAddress(cf));

  Compile_bxx(cf, bgez ? BranchCondition::GreaterEqualZero : BranchCondition::LessThanZero);
}

void CPU::NewRec::Compiler::Compile_blez(CompileFlags cf)
{
  Compile_bxx(cf, BranchCondition::LessEqualZero);
}

void CPU::NewRec::Compiler::Compile_blez_const(CompileFlags cf)
{
  Compile_bxx_const(cf, BranchCondition::LessEqualZero);
}

void CPU::NewRec::Compiler::Compile_bgtz(CompileFlags cf)
{
  Compile_bxx(cf, BranchCondition::GreaterThanZero);
}

void CPU::NewRec::Compiler::Compile_bgtz_const(CompileFlags cf)
{
  Compile_bxx_const(cf, BranchCondition::GreaterThanZero);
}

void CPU::NewRec::Compiler::Compile_beq(CompileFlags cf)
{
  Compile_bxx(cf, BranchCondition::Equal);
}

void CPU::NewRec::Compiler::Compile_beq_const(CompileFlags cf)
{
  Compile_bxx_const(cf, BranchCondition::Equal);
}

void CPU::NewRec::Compiler::Compile_bne(CompileFlags cf)
{
  Compile_bxx(cf, BranchCondition::NotEqual);
}

void CPU::NewRec::Compiler::Compile_bne_const(CompileFlags cf)
{
  Compile_bxx_const(cf, BranchCondition::NotEqual);
}

void CPU::NewRec::Compiler::Compile_bxx_const(CompileFlags cf, BranchCondition cond)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));

  bool taken;
  switch (cond)
  {
    case BranchCondition::Equal:
      taken = GetConstantRegU32(cf.MipsS()) == GetConstantRegU32(cf.MipsT());
      break;

    case BranchCondition::NotEqual:
      taken = GetConstantRegU32(cf.MipsS()) != GetConstantRegU32(cf.MipsT());
      break;

    case BranchCondition::GreaterThanZero:
      taken = GetConstantRegS32(cf.MipsS()) > 0;
      break;

    case BranchCondition::GreaterEqualZero:
      taken = GetConstantRegS32(cf.MipsS()) >= 0;
      break;

    case BranchCondition::LessThanZero:
      taken = GetConstantRegS32(cf.MipsS()) < 0;
      break;

    case BranchCondition::LessEqualZero:
      taken = GetConstantRegS32(cf.MipsS()) <= 0;
      break;

    default:
      Panic("Unhandled condition");
      return;
  }

  const u32 taken_pc = GetConditionalBranchTarget(cf);
  CompileBranchDelaySlot();
  EndBlock(taken ? taken_pc : m_compiler_pc);
}

void CPU::NewRec::Compiler::Compile_sll_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsT()) << inst->r.shamt);
}

void CPU::NewRec::Compiler::Compile_srl_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsT()) >> inst->r.shamt);
}

void CPU::NewRec::Compiler::Compile_sra_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), static_cast<u32>(GetConstantRegS32(cf.MipsT()) >> inst->r.shamt));
}

void CPU::NewRec::Compiler::Compile_sllv_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsT()) << (GetConstantRegU32(cf.MipsS()) & 0x1Fu));
}

void CPU::NewRec::Compiler::Compile_srlv_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsT()) >> (GetConstantRegU32(cf.MipsS()) & 0x1Fu));
}

void CPU::NewRec::Compiler::Compile_srav_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), static_cast<u32>(GetConstantRegS32(cf.MipsT()) >> (GetConstantRegU32(cf.MipsS()) & 0x1Fu)));
}

void CPU::NewRec::Compiler::Compile_and_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) & GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_or_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) | GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_xor_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) ^ GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_nor_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), ~(GetConstantRegU32(cf.MipsS()) | GetConstantRegU32(cf.MipsT())));
}

void CPU::NewRec::Compiler::Compile_slt_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), BoolToUInt32(GetConstantRegS32(cf.MipsS()) < GetConstantRegS32(cf.MipsT())));
}

void CPU::NewRec::Compiler::Compile_sltu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), BoolToUInt32(GetConstantRegU32(cf.MipsS()) < GetConstantRegU32(cf.MipsT())));
}

void CPU::NewRec::Compiler::Compile_mult_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));

  const u64 res =
    static_cast<u64>(static_cast<s64>(GetConstantRegS32(cf.MipsS())) * static_cast<s64>(GetConstantRegS32(cf.MipsT())));
  SetConstantReg(Reg::hi, static_cast<u32>(res >> 32));
  SetConstantReg(Reg::lo, static_cast<u32>(res));
}

void CPU::NewRec::Compiler::Compile_multu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));

  const u64 res = static_cast<u64>(GetConstantRegU32(cf.MipsS())) * static_cast<u64>(GetConstantRegU32(cf.MipsT()));
  SetConstantReg(Reg::hi, static_cast<u32>(res >> 32));
  SetConstantReg(Reg::lo, static_cast<u32>(res));
}

void CPU::NewRec::Compiler::Compile_div_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));

  const s32 num = GetConstantRegS32(cf.MipsS());
  const s32 denom = GetConstantRegS32(cf.MipsT());

  s32 lo, hi;
  if (denom == 0)
  {
    // divide by zero
    lo = (num >= 0) ? UINT32_C(0xFFFFFFFF) : UINT32_C(1);
    hi = static_cast<u32>(num);
  }
  else if (static_cast<u32>(num) == UINT32_C(0x80000000) && denom == -1)
  {
    // unrepresentable
    lo = UINT32_C(0x80000000);
    hi = 0;
  }
  else
  {
    lo = num / denom;
    hi = num % denom;
  }

  SetConstantReg(Reg::hi, hi);
  SetConstantReg(Reg::lo, lo);
}

void CPU::NewRec::Compiler::Compile_divu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));

  const u32 num = GetConstantRegU32(cf.MipsS());
  const u32 denom = GetConstantRegU32(cf.MipsT());

  u32 lo, hi;

  if (denom == 0)
  {
    // divide by zero
    lo = UINT32_C(0xFFFFFFFF);
    hi = static_cast<u32>(num);
  }
  else
  {
    lo = num / denom;
    hi = num % denom;
  }

  SetConstantReg(Reg::hi, hi);
  SetConstantReg(Reg::lo, lo);
}

void CPU::NewRec::Compiler::Compile_add_const(CompileFlags cf)
{
  // TODO: Overflow
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) | GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_addu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) + GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_sub_const(CompileFlags cf)
{
  // TODO: Overflow
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) - GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_subu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()) && HasConstantReg(cf.MipsT()));
  SetConstantReg(MipsD(), GetConstantRegU32(cf.MipsS()) - GetConstantRegU32(cf.MipsT()));
}

void CPU::NewRec::Compiler::Compile_addi_const(CompileFlags cf)
{
  // TODO: Overflow
  DebugAssert(HasConstantReg(cf.MipsS()));
  if (cf.MipsT() != Reg::zero)
    SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) + inst->i.imm_sext32());
}

void CPU::NewRec::Compiler::Compile_addiu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) + inst->i.imm_sext32());
}

void CPU::NewRec::Compiler::Compile_slti_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), BoolToUInt32(GetConstantRegS32(cf.MipsS()) < static_cast<s32>(inst->i.imm_sext32())));
}

void CPU::NewRec::Compiler::Compile_sltiu_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) < inst->i.imm_sext32());
}

void CPU::NewRec::Compiler::Compile_andi_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) & inst->i.imm_zext32());
}

void CPU::NewRec::Compiler::Compile_ori_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) | inst->i.imm_zext32());
}

void CPU::NewRec::Compiler::Compile_xori_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  SetConstantReg(cf.MipsT(), GetConstantRegU32(cf.MipsS()) ^ inst->i.imm_zext32());
}

void CPU::NewRec::Compiler::Compile_lui()
{
  if (inst->i.rt == Reg::zero)
    return;

  SetConstantReg(inst->i.rt, inst->i.imm_zext32() << 16);
}

static constexpr const std::array<std::pair<u32*, u32>, 16> s_cop0_table = {
  {{nullptr, 0x00000000u},
   {nullptr, 0x00000000u},
   {nullptr, 0x00000000u},
   {&CPU::g_state.cop0_regs.BPC, 0xffffffffu},
   {nullptr, 0},
   {&CPU::g_state.cop0_regs.BDA, 0xffffffffu},
   {&CPU::g_state.cop0_regs.TAR, 0x00000000u},
   {&CPU::g_state.cop0_regs.dcic.bits, CPU::Cop0Registers::DCIC::WRITE_MASK},
   {&CPU::g_state.cop0_regs.BadVaddr, 0x00000000u},
   {&CPU::g_state.cop0_regs.BDAM, 0xffffffffu},
   {nullptr, 0x00000000u},
   {&CPU::g_state.cop0_regs.BPCM, 0xffffffffu},
   {&CPU::g_state.cop0_regs.sr.bits, CPU::Cop0Registers::SR::WRITE_MASK},
   {&CPU::g_state.cop0_regs.cause.bits, CPU::Cop0Registers::CAUSE::WRITE_MASK},
   {&CPU::g_state.cop0_regs.EPC, 0x00000000u},
   {&CPU::g_state.cop0_regs.PRID, 0x00000000u}}};

u32* CPU::NewRec::Compiler::GetCop0RegPtr(Cop0Reg reg)
{
  return (static_cast<u8>(reg) < s_cop0_table.size()) ? s_cop0_table[static_cast<u8>(reg)].first : nullptr;
}

u32 CPU::NewRec::Compiler::GetCop0RegWriteMask(Cop0Reg reg)
{
  return (static_cast<u8>(reg) < s_cop0_table.size()) ? s_cop0_table[static_cast<u8>(reg)].second : 0;
}

void CPU::NewRec::Compiler::Compile_mfc0(CompileFlags cf)
{
  const Cop0Reg r = static_cast<Cop0Reg>(MipsD());
  const u32* ptr = GetCop0RegPtr(r);
  if (!ptr)
  {
    Log_ErrorPrintf("Read from unknown cop0 reg %u", static_cast<u32>(r));
    Compile_Fallback();
    return;
  }

  DebugAssert(cf.valid_host_t);
  LoadHostRegFromCPUPointer(cf.host_t, ptr);
}

std::pair<u32*, CPU::NewRec::Compiler::GTERegisterAccessAction>
CPU::NewRec::Compiler::GetGTERegisterPointer(u32 index, bool writing)
{
  if (!writing)
  {
    // Most GTE registers can be read directly. Handle the special cases here.
    if (index == 15) // SXY3
    {
      // mirror of SXY2
      index = 14;
    }

    switch (index)
    {
      case 28: // IRGB
      case 29: // ORGB
      {
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::CallHandler);
      }
      break;

      default:
      {
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::Direct);
      }
      break;
    }
  }
  else
  {
    switch (index)
    {
      case 1:  // V0[z]
      case 3:  // V1[z]
      case 5:  // V2[z]
      case 8:  // IR0
      case 9:  // IR1
      case 10: // IR2
      case 11: // IR3
      case 36: // RT33
      case 44: // L33
      case 52: // LR33
      case 58: // H       - sign-extended on read but zext on use
      case 59: // DQA
      case 61: // ZSF3
      case 62: // ZSF4
      {
        // sign-extend z component of vector registers
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::SignExtend16);
      }
      break;

      case 7:  // OTZ
      case 16: // SZ0
      case 17: // SZ1
      case 18: // SZ2
      case 19: // SZ3
      {
        // zero-extend unsigned values
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::ZeroExtend16);
      }
      break;

      case 15: // SXY3
      {
        // writing to SXYP pushes to the FIFO
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::PushFIFO);
      }
      break;

      case 28: // IRGB
      case 30: // LZCS
      case 63: // FLAG
      {
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::CallHandler);
      }

      case 29: // ORGB
      case 31: // LZCR
      {
        // read-only registers
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::Ignore);
      }

      default:
      {
        // written as-is, 2x16 or 1x32 bits
        return std::make_pair(&g_state.gte_regs.r32[index], GTERegisterAccessAction::Direct);
      }
    }
  }
}

void CPU::NewRec::Compiler::AddGTETicks(TickCount ticks)
{
  // TODO: check, int has +1 here
  m_gte_done_cycle = m_cycles + ticks;
  Log_DebugPrintf("Adding %d GTE ticks", ticks);
}

void CPU::NewRec::Compiler::StallUntilGTEComplete()
{
  // TODO: hack to match old rec.. this may or may not be correct behavior
  // it's the difference between stalling before and after the current instruction's cycle
  DebugAssert(m_cycles > 0);
  m_cycles--;

  if (!m_dirty_gte_done_cycle)
  {
    // simple case - in block scheduling
    if (m_gte_done_cycle > m_cycles)
    {
      Log_DebugPrintf("Stalling for %d ticks from GTE", m_gte_done_cycle - m_cycles);
      m_cycles += (m_gte_done_cycle - m_cycles);
    }
  }
  else
  {
    // switch to in block scheduling
    Log_DebugPrintf("Flushing GTE stall from state");
    Flush(FLUSH_GTE_STALL_FROM_STATE);
  }

  m_cycles++;
}
