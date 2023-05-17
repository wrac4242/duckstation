// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "cpu_newrec_compiler.h"
#include "common/assert.h"
#include "common/log.h"
#include "common/string.h"
#include "cpu_core_private.h"
#include "cpu_disasm.h"
#include "cpu_newrec_private.h"
#include "settings.h"
#include <cstdint>
#include <limits>
Log_SetChannel(NewRec::Compiler);

CPU::NewRec::Compiler::Compiler() = default;

CPU::NewRec::Compiler::~Compiler() = default;

const char* CPU::NewRec::Compiler::GetHostRegName(u32 reg) const
{
  static constexpr std::array<const char*, 16> reg64_names = {
    {"rax", "rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"}};
  return (reg < reg64_names.size()) ? reg64_names[reg] : "UNKNOWN";
}

void CPU::NewRec::Compiler::Reset(Block* block)
{
  m_block = block;
  m_compiler_pc = block->pc;
  m_cycles = 0;
  inst = nullptr;
  m_current_instruction_pc = 0;
  m_current_instruction_branch_delay_slot = false;
  m_dirty_pc = false;
  m_dirty_instruction_bits = false;
  m_constant_reg_values.fill(0);
  m_constant_regs_valid.reset();
  m_constant_regs_dirty.reset();

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
    ClearHostReg(i);
  m_register_alloc_counter = 0;

  m_constant_reg_values[static_cast<u32>(Reg::zero)] = 0;
  m_constant_regs_valid.set(static_cast<u32>(Reg::zero));

  m_load_delay_dirty = true;
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
  Reset(block);

  Log_DebugPrintf("Block range: %08X -> %08X", m_block->pc, block->pc + block->size * sizeof(Instruction));

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

  const auto [code, size] = EndCompile();

#if 0
  Log_DebugPrintf("Whole block for %08X...", block->pc);
  DisassembleAndLog(code, size);
  Log_DebugPrintf(" --- %u bytes", size);
#else
  Log_DebugPrintf("Whole block for %08X took %u bytes", block->pc, size);
#endif

  g_code_buffer.CommitCode(size);

  return code;
}

void CPU::NewRec::Compiler::SetConstantReg(Reg r, u32 v)
{
  DebugAssert(r < Reg::count && r != Reg::zero);
  if (m_constant_regs_valid.test(static_cast<u32>(r)) && m_constant_reg_values[static_cast<u32>(r)] == v)
    return;

  m_constant_reg_values[static_cast<u32>(r)] = v;
  m_constant_regs_valid.set(static_cast<u32>(r));
  m_constant_regs_dirty.set(static_cast<u32>(r));

  if (const std::optional<u32> hostreg = CheckHostReg(0, HR_TYPE_CPU_REG, static_cast<s8>(r)); hostreg.has_value())
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
    Log_DebugPrintf("Invalidating non-dirty registers, and flushing load delay from state");
    Flush(FLUSH_INVALIDATE_NON_DIRTY_GUEST_REGISTERS | FLUSH_LOAD_DELAY_FROM_STATE);
  }

  // commit the delayed register load
  FinishLoadDelay();

  // move next load delay forward
  if (m_next_load_delay_register != Reg::count)
  {
    // if it somehow got flushed, read it back in.
    if (m_next_load_delay_value_register == NUM_HOST_REGS)
    {
      AllocateHostReg(HR_MODE_READ, HR_TYPE_NEXT_LOAD_DELAY_VALUE, static_cast<u8>(m_next_load_delay_register));
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
    AllocateHostReg(HR_MODE_READ, HR_TYPE_LOAD_DELAY_VALUE, static_cast<s8>(m_load_delay_register));
    DebugAssert(m_load_delay_value_register != NUM_HOST_REGS);
  }

  // kill any (old) cached value for this register
  DeleteGuestReg(m_load_delay_register, false);

  Log_DebugPrintf("Finished delayed load to %s in host register %s", GetRegName(m_load_delay_register),
                  GetHostRegName(m_load_delay_value_register));

  // and swap the mode over so it gets written back later
  HostRegAlloc& ra = m_host_regs[m_load_delay_value_register];
  DebugAssert(ra.reg == static_cast<s8>(m_load_delay_register));
  ra.flags = (ra.flags & IMMUTABLE_HR_FLAGS) | HR_ALLOCATED | HR_MODE_READ | HR_MODE_WRITE;
  ra.counter = m_register_alloc_counter++;
  ra.type = HR_TYPE_CPU_REG;

  // constants are gone
  Log_DebugPrintf("Clearing constant in %s due to load delay", GetRegName(m_load_delay_register));
  ClearConstantReg(m_load_delay_register);

  m_load_delay_register = Reg::count;
  m_load_delay_value_register = NUM_HOST_REGS;
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

u32 CPU::NewRec::Compiler::GetConditionalBranchTarget() const
{
  return m_compiler_pc + (inst->i.imm_sext32() << 2);
}

u32 CPU::NewRec::Compiler::GetBranchReturnAddress() const
{
  return m_compiler_pc + sizeof(Instruction);
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
  u32 lowest = NUM_HOST_REGS;
  u16 lowest_count = std::numeric_limits<u16>::max();
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    const HostRegAlloc& ra = m_host_regs[i];
    if (!(ra.flags & HR_USABLE))
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
                      GetRegName(static_cast<Reg>(ra.reg)));
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

u32 CPU::NewRec::Compiler::AllocateHostReg(u32 flags, HostRegAllocType type /* = HOST_REG_ALLOC_TYPE_TEMP */,
                                           s8 reg /* = -1 */)
{
  // Cancel any load delays before booting anything out
  if (flags & HR_MODE_WRITE && (type == HR_TYPE_CPU_REG || type == HR_TYPE_NEXT_LOAD_DELAY_VALUE))
    CancelLoadDelaysToReg(static_cast<Reg>(reg));

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
      DebugAssert(reg != static_cast<s8>(Reg::zero));

      Log_DebugPrintf("Allocate host reg %s to guest reg %s in %s mode", GetHostRegName(hreg.value()),
                      GetRegName(static_cast<Reg>(reg)), GetReadWriteModeString(flags));

      if (flags & HR_MODE_READ)
      {
        PopulateHostReg(hreg.value());

        if (HasConstantReg(static_cast<Reg>(reg)))
        {
          // may as well flush it now
          Log_DebugPrintf("Flush constant register in guest reg %s to host reg %s", GetRegName(static_cast<Reg>(reg)),
                          GetHostRegName(hreg.value()));
          m_constant_regs_dirty.reset(reg);
          ra.flags |= HR_MODE_WRITE;
        }
      }

      if (flags & HR_MODE_WRITE && HasConstantReg(static_cast<Reg>(reg)))
      {
        DebugAssert(reg != static_cast<s8>(Reg::zero));
        Log_DebugPrintf("Clearing constant register in guest reg %s due to write mode in %s",
                        GetRegName(static_cast<Reg>(reg)), GetHostRegName(hreg.value()));

        ClearConstantReg(static_cast<Reg>(reg));
      }
    }
    break;

    case HR_TYPE_LOAD_DELAY_VALUE:
    {
      DebugAssert(!m_load_delay_dirty && (!HasLoadDelay() || !(flags & HR_MODE_WRITE)));
      Log_DebugPrintf("Allocating load delayed guest register %s in host reg %s in %s mode",
                      GetRegName(static_cast<Reg>(reg)), GetHostRegName(hreg.value()), GetReadWriteModeString(flags));
      m_load_delay_register = static_cast<Reg>(reg);
      m_load_delay_value_register = hreg.value();
      if (flags & HR_MODE_READ)
        PopulateHostReg(hreg.value());
    }
    break;

    case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
    {
      Log_DebugPrintf("Allocating next load delayed guest register %s in host reg %s in %s mode",
                      GetRegName(static_cast<Reg>(reg)), GetHostRegName(hreg.value()), GetReadWriteModeString(flags));
      m_next_load_delay_register = static_cast<Reg>(reg);
      m_next_load_delay_value_register = hreg.value();
      if (flags & HR_MODE_READ)
        PopulateHostReg(hreg.value());
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
                                                       s8 reg /* = -1 */)
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
        Log_DebugPrintf("Switch guest reg %s from read to read-write in host reg %s", GetRegName(static_cast<Reg>(reg)),
                        GetHostRegName(i));
      }

      if (HasConstantReg(static_cast<Reg>(reg)))
      {
        DebugAssert(reg != static_cast<s8>(Reg::zero));
        Log_DebugPrintf("Clearing constant register in guest reg %s due to write mode in %s",
                        GetRegName(static_cast<Reg>(reg)), GetHostRegName(i));

        ClearConstantReg(static_cast<Reg>(reg));
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

void CPU::NewRec::Compiler::FlushHostReg(u32 reg)
{
  HostRegAlloc& ra = m_host_regs[reg];
  if (ra.flags & HR_MODE_WRITE)
  {
    switch (ra.type)
    {
      case HR_TYPE_CPU_REG:
      {
        DebugAssert(ra.reg > 0 && ra.reg < static_cast<s8>(Reg::count));
        Log_DebugPrintf("Flushing register %s in host register %s to state", GetRegName(static_cast<Reg>(ra.reg)),
                        GetHostRegName(reg));
      }
      break;

      case HR_TYPE_LOAD_DELAY_VALUE:
      {
        DebugAssert(m_load_delay_value_register == reg);
        Log_DebugPrintf("Flushing load delayed register %s in host register %s to state",
                        GetRegName(static_cast<Reg>(ra.reg)), GetHostRegName(reg));

        m_load_delay_value_register = NUM_HOST_REGS;
      }
      break;

      case HR_TYPE_NEXT_LOAD_DELAY_VALUE:
      {
        DebugAssert(m_next_load_delay_value_register == reg);
        Log_WarningPrintf("Flushing NEXT load delayed register %s in host register %s to state",
                          GetRegName(static_cast<Reg>(ra.reg)), GetHostRegName(reg));

        m_next_load_delay_value_register = NUM_HOST_REGS;
      }
      break;
    }

    WritebackHostReg(reg);
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
  ra.reg = -1;
}

void CPU::NewRec::Compiler::MarkRegsNeeded(HostRegAllocType type, s8 reg)
{
  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (ra.flags & HR_ALLOCATED && ra.type == type && ra.reg == reg)
      ra.flags |= HR_NEEDED;
  }
}

void CPU::NewRec::Compiler::RenameHostReg(u32 reg, u32 new_flags, HostRegAllocType new_type, s8 new_reg)
{
  // only supported for cpu regs for now
  DebugAssert(new_type == HR_TYPE_TEMP || new_type == HR_TYPE_CPU_REG);

  const std::optional<u32> old_reg = CheckHostReg(0, new_type, new_reg);
  if (old_reg.has_value())
  {
    // don't writeback
    ClearHostReg(old_reg.value());
  }

  // kill any load delay to this reg
  if (new_type == HR_TYPE_CPU_REG)
    CancelLoadDelaysToReg(static_cast<Reg>(new_reg));

  if (new_type == HR_TYPE_CPU_REG)
    Log_DebugPrintf("Renaming host reg %s to guest reg %s", GetHostRegName(reg), GetRegName(static_cast<Reg>(new_reg)));
  else
    Log_DebugPrintf("Renaming host reg %s");

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

    if (ra.flags & HR_MODE_WRITE)
      ra.flags |= HR_MODE_READ;

    ra.flags &= ~HR_NEEDED;
  }
}

void CPU::NewRec::Compiler::DeleteGuestReg(Reg reg, bool flush)
{
  DebugAssert(reg != Reg::zero);

  for (u32 i = 0; i < NUM_HOST_REGS; i++)
  {
    HostRegAlloc& ra = m_host_regs[i];
    if (ra.flags & HR_ALLOCATED && ra.type == HR_TYPE_CPU_REG && ra.reg == static_cast<s8>(reg))
    {
      if (flush)
        WritebackHostReg(i);
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
  if (flags & (FLUSH_FREE_CALLER_SAVED_REGISTERS | FLUSH_FREE_ALL_REGISTERS))
  {
    const u32 req_mask = (flags & FLUSH_FREE_ALL_REGISTERS) ? HR_ALLOCATED : (HR_ALLOCATED | HR_CALLEE_SAVED);
    constexpr u32 req_flags = HR_ALLOCATED;

    for (u32 i = 0; i < NUM_HOST_REGS; i++)
    {
      HostRegAlloc& ra = m_host_regs[i];
      if ((ra.flags & req_mask) == req_flags)
        FreeHostReg(i);
    }
  }

  if (flags & FLUSH_INVALIDATE_GUEST_REGISTERS)
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
    if (flags & FLUSH_FLUSH_GUEST_REGISTERS)
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

    if (flags & FLUSH_INVALIDATE_NON_DIRTY_GUEST_REGISTERS)
    {
      constexpr u32 req_flags = (HR_ALLOCATED | HR_MODE_WRITE);

      for (u32 i = 0; i < NUM_HOST_REGS; i++)
      {
        HostRegAlloc& ra = m_host_regs[i];
        if (ra.type != HR_TYPE_CPU_REG || ((ra.flags & req_flags) != req_flags))
          continue;

        Log_DebugPrintf("Freeing non-dirty cached register %s in %s", GetRegName(static_cast<Reg>(ra.reg)),
                        GetHostRegName(i));
        FreeHostReg(i);
      }

      // remove any non-dirty constants too
      for (u32 i = 1; i < static_cast<u32>(Reg::count); i++)
      {
        if (!HasConstantReg(static_cast<Reg>(i)) || HasDirtyConstantReg(static_cast<Reg>(i)))
          continue;

        Log_DebugPrintf("Clearing non-dirty constant %s", GetRegName(static_cast<Reg>(i)));
        ClearConstantReg(static_cast<Reg>(i));
      }
    }
  }
}

void CPU::NewRec::Compiler::FlushConstantReg(Reg r)
{
  DebugAssert(m_constant_regs_valid.test(static_cast<u32>(r)));
  m_constant_regs_dirty.reset(static_cast<u32>(r));
}

void CPU::NewRec::Compiler::BackupHostState()
{
  DebugAssert(m_host_state_backup_count < m_host_state_backup.size());

  // need to back up everything...
  HostStateBackup& bu = m_host_state_backup[m_host_state_backup_count];
  bu.cycles = m_cycles;
  bu.compiler_pc = m_compiler_pc;
  bu.dirty_pc = m_dirty_pc;
  bu.dirty_instruction_bits = m_dirty_instruction_bits;
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
  m_dirty_instruction_bits = bu.dirty_instruction_bits;
  m_dirty_pc = bu.dirty_pc;
  m_compiler_pc = bu.compiler_pc;
  m_register_alloc_counter = bu.register_alloc_counter;
  m_load_delay_dirty = bu.load_delay_dirty;
  m_load_delay_register = bu.load_delay_register;
  m_load_delay_value_register = bu.load_delay_value_register;
  m_next_load_delay_register = bu.next_load_delay_register;
  m_next_load_delay_value_register = bu.next_load_delay_value_register;
  m_cycles = bu.cycles;
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
      case InstructionFunct::jalr: CompileTemplate(&Compiler::Compile_jalr_const, &Compiler::Compile_jalr, TF_WRITES_D | TF_READS_S | TF_NO_NOP); break;
      case InstructionFunct::syscall: Compile_syscall(); break;
      case InstructionFunct::break_: Compile_break(); break;
      case InstructionFunct::mfhi: CompileMoveRegTemplate(inst->r.rd, Reg::hi); break;
      case InstructionFunct::mthi: CompileMoveRegTemplate(Reg::hi, inst->r.rs); break;
      case InstructionFunct::mflo: CompileMoveRegTemplate(inst->r.rd, Reg::lo); break;
      case InstructionFunct::mtlo: CompileMoveRegTemplate(Reg::lo, inst->r.rs); break;
      case InstructionFunct::mult: CompileTemplate(&Compiler::Compile_mult_const, &Compiler::Compile_mult, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI); break;
      case InstructionFunct::multu: CompileTemplate(&Compiler::Compile_multu_const, &Compiler::Compile_multu, TF_READS_S | TF_READS_T | TF_WRITES_LO | TF_WRITES_HI); break;
      case InstructionFunct::div: Compile_Fallback(); break;
      case InstructionFunct::divu: Compile_Fallback(); break;
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

    case InstructionOp::b: CompileTemplate(&Compiler::Compile_b_const, &Compiler::Compile_b, TF_READS_S); break;
    case InstructionOp::blez: CompileTemplate(&Compiler::Compile_blez_const, &Compiler::Compile_blez, TF_READS_S); break;
    case InstructionOp::bgtz: CompileTemplate(&Compiler::Compile_bgtz_const, &Compiler::Compile_bgtz, TF_READS_S); break;
    case InstructionOp::beq: CompileTemplate(&Compiler::Compile_beq_const, &Compiler::Compile_beq, TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;
    case InstructionOp::bne: CompileTemplate(&Compiler::Compile_bne_const, &Compiler::Compile_bne, TF_READS_S | TF_READS_T | TF_COMMUTATIVE); break;

    case InstructionOp::addi: CompileTemplate(&Compiler::Compile_addi_const, &Compiler::Compile_addi, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE | TF_CAN_OVERFLOW); break;
    case InstructionOp::addiu: CompileTemplate(&Compiler::Compile_addiu_const, &Compiler::Compile_addiu, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::slti: CompileTemplate(&Compiler::Compile_slti_const, &Compiler::Compile_slti, TF_WRITES_T | TF_READS_S); break;
    case InstructionOp::sltiu: CompileTemplate(&Compiler::Compile_sltiu_const, &Compiler::Compile_sltiu, TF_WRITES_T | TF_READS_S); break;
    case InstructionOp::andi: CompileTemplate(&Compiler::Compile_andi_const, &Compiler::Compile_andi, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::ori: CompileTemplate(&Compiler::Compile_ori_const, &Compiler::Compile_ori, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::xori: CompileTemplate(&Compiler::Compile_xori_const, &Compiler::Compile_xori, TF_WRITES_T | TF_READS_S | TF_COMMUTATIVE); break;
    case InstructionOp::lui: Compile_lui(); break;

    case InstructionOp::lb: CompileLoadStoreTemplate(MemoryAccessSize::Byte, true, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lbu: CompileLoadStoreTemplate(MemoryAccessSize::Byte, false, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lh: CompileLoadStoreTemplate(MemoryAccessSize::HalfWord, true, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lhu: CompileLoadStoreTemplate(MemoryAccessSize::HalfWord, false, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lw: CompileLoadStoreTemplate(MemoryAccessSize::Word, true, TF_READS_S | TF_WRITES_T | TF_LOAD_DELAY); break;
    case InstructionOp::lwl: Compile_Fallback(); break;
    case InstructionOp::lwr: Compile_Fallback(); break;
    case InstructionOp::sb: CompileLoadStoreTemplate(MemoryAccessSize::Byte, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::sh: CompileLoadStoreTemplate(MemoryAccessSize::HalfWord, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::sw: CompileLoadStoreTemplate(MemoryAccessSize::Word, false, TF_READS_S | TF_READS_T); break;
    case InstructionOp::swl: Compile_Fallback(); break;
    case InstructionOp::swr: Compile_Fallback(); break;

    case InstructionOp::cop0: Compile_Fallback(); break;

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

void CPU::NewRec::Compiler::CompileTemplate(void (Compiler::*const_func)(CompileFlags cf),
                                            void (Compiler::*func)(CompileFlags cf), u32 tflags)
{
  // TODO: This is where we will do memory operand optimization. Remember to kill constants!
  // TODO: Swap S and T if commutative
  // TODO: For and, treat as zeroing if imm is zero
  // TODO: Optimize slt + bne to cmp + jump
  // TODO: Rename S to D for sll 0, addi 0, etc.
  // TODO: Prefer memory operands when load delay is dirty, since we're going to invalidate immediately after the first
  // instruction..

  bool allow_constant = static_cast<bool>(const_func);
  Reg rs = inst->r.rs.GetValue();
  Reg rt = inst->r.rt.GetValue();
  Reg rd = inst->r.rd.GetValue();

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
    MarkRegsNeeded(HR_TYPE_CPU_REG, static_cast<s8>(rs));
    if (HasConstantReg(rs))
      cf.const_s = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_T)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, static_cast<s8>(rt));
    if (HasConstantReg(rt))
      cf.const_t = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_LO)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, static_cast<s8>(Reg::lo));
    if (HasConstantReg(Reg::lo))
      cf.const_lo = true;
    else
      allow_constant = false;
  }
  if (tflags & TF_READS_HI)
  {
    MarkRegsNeeded(HR_TYPE_CPU_REG, static_cast<s8>(Reg::hi));
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

  if (tflags & TF_READS_S &&
      (tflags & TF_NEEDS_REG_S || !cf.const_s || (tflags & TF_WRITES_D && rd != Reg::zero && rd == rs)))
  {
    cf.host_s = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rs));
    cf.const_s = false;
    cf.valid_host_s = true;
  }

  if (tflags & TF_READS_T &&
      (tflags & (TF_NEEDS_REG_T | TF_WRITES_T) || !cf.const_t || (tflags & TF_WRITES_D && rd != Reg::zero && rd == rt)))
  {
    cf.host_t = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rt));
    cf.const_t = false;
    cf.valid_host_t = true;
  }

  if (tflags & (TF_READS_LO | TF_WRITES_LO))
  {
    cf.host_lo =
      AllocateHostReg(((tflags & TF_READS_LO) ? HR_MODE_READ : 0u) | ((tflags & TF_WRITES_LO) ? HR_MODE_WRITE : 0u),
                      HR_TYPE_CPU_REG, static_cast<s8>(Reg::lo));
    cf.const_lo = false;
    cf.valid_host_lo = true;
  }

  if (tflags & (TF_READS_HI | TF_WRITES_HI))
  {
    cf.host_hi =
      AllocateHostReg(((tflags & TF_READS_HI) ? HR_MODE_READ : 0u) | ((tflags & TF_WRITES_HI) ? HR_MODE_WRITE : 0u),
                      HR_TYPE_CPU_REG, static_cast<s8>(Reg::hi));
    cf.const_hi = false;
    cf.valid_host_hi = true;
  }

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
      DeleteGuestReg(rd, false);
      RenameHostReg(tempreg, HR_MODE_WRITE, HR_TYPE_CPU_REG, static_cast<s8>(rd));
    }
    else if (tflags & TF_WRITES_T && rt != Reg::zero)
    {
      DeleteGuestReg(rt, false);
      RenameHostReg(tempreg, HR_MODE_WRITE, HR_TYPE_CPU_REG, static_cast<s8>(rt));
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
      DebugAssert(!(tflags & TF_LOAD_DELAY));
      cf.host_d = AllocateHostReg(HR_MODE_WRITE, HR_TYPE_CPU_REG, static_cast<s8>(rd));
      cf.valid_host_d = true;
    }

    if (tflags & TF_WRITES_T && rt != Reg::zero)
    {
      DebugAssert(!(tflags & TF_LOAD_DELAY));
      cf.host_t = AllocateHostReg(HR_MODE_WRITE, HR_TYPE_CPU_REG, static_cast<s8>(rt));
      cf.valid_host_t = true;
    }

    (this->*func)(cf);
  }
}

void CPU::NewRec::Compiler::CompileLoadStoreTemplate(MemoryAccessSize size, bool sign, u32 tflags)
{
  const Reg rs = inst->i.rs;
  const Reg rt = inst->i.rt;

  CompileFlags cf = {};

  if (tflags & TF_READS_S)
    cf.mips_s = static_cast<u8>(rs);
  if (tflags & (TF_READS_T | TF_WRITES_T))
    cf.mips_t = static_cast<u8>(rt);

  // constant address?
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
      const std::optional<u32> hreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rs));
      if (hreg.has_value())
      {
        cf.valid_host_s = true;
        cf.host_s = hreg.value();
      }
    }
    else
    {
      // need rs in a register
      cf.host_s = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rs));
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
        const std::optional<u32> hreg = CheckHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rt));
        if (hreg.has_value())
        {
          cf.valid_host_t = true;
          cf.host_t = hreg.value();
        }
      }
      else
      {
        cf.host_t = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(rt));
        cf.valid_host_t = true;
      }
    }

    Compile_Store(cf, size, std::move(addr));
  }
  else
  {
    // invalidate T, we're overwriting it
    // TODO: only if we're not emulating load delays
#if 0
    if (rt != Reg::zero && rs != rt)
      DeleteGuestReg(rt, false);
#endif

    Compile_Load(cf, size, sign, std::move(addr));
  }
}

void CPU::NewRec::Compiler::CompileMoveRegTemplate(Reg dst, Reg src)
{
  if (dst == src || dst == Reg::zero)
    return;

  if (HasConstantReg(src))
  {
    DeleteGuestReg(dst, false);
    SetConstantReg(dst, GetConstantRegU32(src));
    return;
  }

  // TODO: rename if src is no longer used
  const u32 srcreg = AllocateHostReg(HR_MODE_READ, HR_TYPE_CPU_REG, static_cast<s8>(src));
  const u32 dstreg = AllocateHostReg(HR_MODE_WRITE, HR_TYPE_CPU_REG, static_cast<s8>(dst));
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
  SetConstantReg(Reg::ra, GetBranchReturnAddress());
  CompileBranchDelaySlot();
  EndBlock(newpc);
}

void CPU::NewRec::Compiler::Compile_jalr_const(CompileFlags cf)
{
  DebugAssert(HasConstantReg(cf.MipsS()));
  const u32 newpc = GetConstantRegU32(cf.MipsS());
  if (MipsD() != Reg::zero)
    SetConstantReg(MipsD(), GetBranchReturnAddress());
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
  const u32 taken_pc = GetConditionalBranchTarget();

  if (link)
    SetConstantReg(Reg::ra, GetBranchReturnAddress());

  CompileBranchDelaySlot();
  EndBlock(taken ? taken_pc : m_compiler_pc);
}

void CPU::NewRec::Compiler::Compile_b(CompileFlags cf)
{
  const u8 irt = static_cast<u8>(inst->i.rt.GetValue());
  const bool bgez = ConvertToBoolUnchecked(irt & u8(1));
  const bool link = (irt & u8(0x1E)) == u8(0x10);

  if (link)
    SetConstantReg(Reg::ra, GetBranchReturnAddress());

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

  const u32 taken_pc = GetConditionalBranchTarget();
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
