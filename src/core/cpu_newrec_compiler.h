// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "common/platform.h"
#include "cpu_newrec.h"
#include "cpu_newrec_private.h"
#include "cpu_types.h"
#include <array>
#include <bitset>
#include <optional>
#include <utility>
#include <vector>

namespace CPU::NewRec {

static constexpr u32 NUM_HOST_REGS = 16;
static constexpr bool HAS_MEMORY_OPERANDS = true;

class Compiler
{
public:
  Compiler();
  virtual ~Compiler();

  const void* CompileBlock(Block* block);

protected:
  enum FlushFlags : u32
  {
    FLUSH_FLUSH_MIPS_REGISTERS = (1 << 0),
    FLUSH_INVALIDATE_MIPS_REGISTERS = (1 << 1),
    FLUSH_INVALIDATE_NON_DIRTY_MIPS_REGISTERS = (1 << 2),
    FLUSH_FREE_CALLER_SAVED_REGISTERS = (1 << 3),
    FLUSH_FREE_ALL_REGISTERS = (1 << 4),
    FLUSH_PC = (1 << 5),
    FLUSH_INSTRUCTION_BITS = (1 << 6),
    FLUSH_CYCLES = (1 << 7),
    FLUSH_LOAD_DELAY = (1 << 8),
    FLUSH_LOAD_DELAY_FROM_STATE = (1 << 9),

    FLUSH_FOR_C_CALL = (FLUSH_FREE_CALLER_SAVED_REGISTERS),
    FLUSH_FOR_LOADSTORE = (FLUSH_FREE_CALLER_SAVED_REGISTERS | FLUSH_CYCLES),
    FLUSH_FOR_BRANCH = (FLUSH_FLUSH_MIPS_REGISTERS),
    FLUSH_FOR_INTERPRETER =
      (FLUSH_FLUSH_MIPS_REGISTERS | FLUSH_INVALIDATE_MIPS_REGISTERS | FLUSH_FREE_CALLER_SAVED_REGISTERS | FLUSH_PC |
       FLUSH_CYCLES | FLUSH_INSTRUCTION_BITS | FLUSH_LOAD_DELAY),
    FLUSH_END_BLOCK = 0xFFFFFFFFu & ~(FLUSH_PC | FLUSH_INSTRUCTION_BITS),
  };

  union CompileFlags
  {
    struct
    {
      u32 const_s : 1;  // S is constant
      u32 const_t : 1;  // T is constant
      u32 const_lo : 1; // LO is constant
      u32 const_hi : 1; // HI is constant

      u32 valid_host_d : 1;  // D is valid in host register
      u32 valid_host_s : 1;  // S is valid in host register
      u32 valid_host_t : 1;  // T is valid in host register
      u32 valid_host_lo : 1; // LO is valid in host register
      u32 valid_host_hi : 1; // HI is valid in host register

      u32 delay_slot_swapped : 1;

      u32 host_d : 5;  // D host register
      u32 host_s : 5;  // S host register
      u32 host_t : 5;  // T host register
      u32 host_lo : 5; // LO host register

      u32 pad1 : 2; // 28..31

      u32 host_hi : 5; // HI host register

      u32 mips_s : 5; // S guest register
      u32 mips_t : 5; // T guest register

      u32 pad2 : 15; // 32 bits
    };

    u64 bits;

    ALWAYS_INLINE Reg MipsS() const { return static_cast<Reg>(mips_s); }
    ALWAYS_INLINE Reg MipsT() const { return static_cast<Reg>(mips_t); }
  };
  static_assert(sizeof(CompileFlags) == sizeof(u64));

  enum TemplateFlag : u32
  {
    TF_READS_S = (1 << 0),
    TF_READS_T = (1 << 1),
    TF_READS_D = (1 << 2),
    TF_READS_LO = (1 << 3),
    TF_READS_HI = (1 << 4),
    TF_WRITES_D = (1 << 5),
    TF_WRITES_T = (1 << 6),
    TF_WRITES_LO = (1 << 7),
    TF_WRITES_HI = (1 << 8),
    TF_COMMUTATIVE = (1 << 9), // S op T == T op S
    TF_CAN_OVERFLOW = (1 << 10),

    // TF_NORENAME = // TODO
    // TF_FORCEREGS
    // TF_FORCEREGT
    TF_LOAD_DELAY = (1 << 11),
    TF_GTE_STALL = (1 << 12),

    TF_NO_NOP = (1 << 13),
    TF_NEEDS_REG_S = (1 << 14),
    TF_NEEDS_REG_T = (1 << 15),
  };

  enum HostRegFlags : u8
  {
    HR_ALLOCATED = (1 << 0),
    HR_NEEDED = (1 << 1),
    HR_MODE_READ = (1 << 2),  // valid
    HR_MODE_WRITE = (1 << 3), // dirty

    HR_USABLE = (1 << 7),
    HR_CALLEE_SAVED = (1 << 6),

    ALLOWED_HR_FLAGS = HR_MODE_READ | HR_MODE_WRITE,
    IMMUTABLE_HR_FLAGS = HR_USABLE | HR_CALLEE_SAVED,
  };

  enum HostRegAllocType : u8
  {
    HR_TYPE_TEMP,
    HR_TYPE_CPU_REG,
    HR_TYPE_PC_WRITEBACK,
    HR_TYPE_LOAD_DELAY_VALUE,
    HR_TYPE_NEXT_LOAD_DELAY_VALUE,
  };

  struct HostRegAlloc
  {
    u8 flags;
    HostRegAllocType type;
    Reg reg;
    u16 counter;
  };

  enum class BranchCondition : u8
  {
    Equal,
    NotEqual,
    GreaterThanZero,
    GreaterEqualZero,
    LessThanZero,
    LessEqualZero,
  };

  ALWAYS_INLINE bool HasConstantReg(Reg r) const { return m_constant_regs_valid.test(static_cast<u32>(r)); }
  ALWAYS_INLINE bool HasDirtyConstantReg(Reg r) const { return m_constant_regs_dirty.test(static_cast<u32>(r)); }
  ALWAYS_INLINE bool HasConstantRegValue(Reg r, u32 val) const
  {
    return m_constant_regs_valid.test(static_cast<u32>(r)) && m_constant_reg_values[static_cast<u32>(r)] == val;
  }
  ALWAYS_INLINE u32 GetConstantRegU32(Reg r) const { return m_constant_reg_values[static_cast<u32>(r)]; }
  ALWAYS_INLINE s32 GetConstantRegS32(Reg r) const
  {
    return static_cast<s32>(m_constant_reg_values[static_cast<u32>(r)]);
  }
  void SetConstantReg(Reg r, u32 v);
  void ClearConstantReg(Reg r);
  void FlushConstantReg(Reg r);
  void FlushConstantRegs(bool invalidate);

  Reg MipsD() const;
  u32 GetConditionalBranchTarget() const;
  u32 GetBranchReturnAddress() const;
  void SetCompilerPC(u32 newpc);

  virtual void DisassembleAndLog(const void* start, u32 size) = 0;

  virtual const void* GetCurrentCodePointer() = 0;

  virtual void Reset(Block* block);
  virtual void BeginBlock();
  virtual void EndBlock(std::optional<u32> newpc) = 0;
  virtual void EndBlockWithException(Exception excode) = 0;
  virtual std::pair<const void*, u32> EndCompile() = 0;

  ALWAYS_INLINE bool IsHostRegAllocated(u32 r) const { return (m_host_regs[r].flags & HR_ALLOCATED) != 0; }
  static const char* GetReadWriteModeString(u32 flags);
  virtual const char* GetHostRegName(u32 reg) const;
  std::optional<u32> GetFreeHostReg(u32 flags);
  u32 AllocateHostReg(u32 flags, HostRegAllocType type = HR_TYPE_TEMP, Reg reg = Reg::count);
  std::optional<u32> CheckHostReg(u32 flags, HostRegAllocType type = HR_TYPE_TEMP, Reg reg = Reg::count);
  void FlushHostReg(u32 reg);
  void FreeHostReg(u32 reg);
  void ClearHostReg(u32 reg);
  void MarkRegsNeeded(HostRegAllocType type, Reg reg);
  void RenameHostReg(u32 reg, u32 new_flags, HostRegAllocType new_type, Reg new_reg);
  void ClearHostRegNeeded(u32 reg);
  void ClearHostRegsNeeded();
  void DeleteMIPSReg(Reg reg, bool flush);

  virtual void LoadHostRegWithConstant(u32 reg, u32 val) = 0;
  virtual void LoadHostRegFromCPUPointer(u32 reg, const void* ptr) = 0;
  virtual void StoreConstantToCPUPointer(u32 val, const void* ptr) = 0;
  virtual void StoreHostRegToCPUPointer(u32 reg, const void* ptr) = 0;
  virtual void CopyHostReg(u32 dst, u32 src) = 0;
  virtual void Flush(u32 flags);

  /// Returns true if there is a load delay which will be stored at the end of the instruction.
  bool HasLoadDelay() const { return m_load_delay_register != Reg::count; }

  /// Cancels any pending load delay to the specified register.
  void CancelLoadDelaysToReg(Reg reg);

  /// Moves load delay to the next load delay, and writes any previous load delay to the destination register.
  void UpdateLoadDelay();

  /// Flushes the load delay, i.e. writes it to the destination register.
  void FinishLoadDelay();

  void BackupHostState();
  void RestoreHostState();

  void CompileInstruction();
  void CompileBranchDelaySlot(bool dirty_pc = true);

  void CompileTemplate(void (Compiler::*const_func)(CompileFlags cf), void (Compiler::*func)(CompileFlags cf),
                       u32 tflags);
  void CompileLoadStoreTemplate(MemoryAccessSize size, bool store, bool sign, u32 tflags);
  void CompileMoveRegTemplate(Reg dst, Reg src);

  virtual void Compile_Fallback() = 0;

  void Compile_j();
  virtual void Compile_jr(CompileFlags cf) = 0;
  void Compile_jr_const(CompileFlags cf);
  void Compile_jal();
  virtual void Compile_jalr(CompileFlags cf) = 0;
  void Compile_jalr_const(CompileFlags cf);
  void Compile_syscall();
  void Compile_break();

  void Compile_b_const(CompileFlags cf);
  void Compile_b(CompileFlags cf);
  void Compile_blez(CompileFlags cf);
  void Compile_blez_const(CompileFlags cf);
  void Compile_bgtz(CompileFlags cf);
  void Compile_bgtz_const(CompileFlags cf);
  void Compile_beq(CompileFlags cf);
  void Compile_beq_const(CompileFlags cf);
  void Compile_bne(CompileFlags cf);
  void Compile_bne_const(CompileFlags cf);
  virtual void Compile_bxx(CompileFlags cf, BranchCondition cond) = 0;
  void Compile_bxx_const(CompileFlags cf, BranchCondition cond);

  void Compile_sll_const(CompileFlags cf);
  virtual void Compile_sll(CompileFlags cf) = 0;
  void Compile_srl_const(CompileFlags cf);
  virtual void Compile_srl(CompileFlags cf) = 0;
  void Compile_sra_const(CompileFlags cf);
  virtual void Compile_sra(CompileFlags cf) = 0;
  void Compile_sllv_const(CompileFlags cf);
  virtual void Compile_sllv(CompileFlags cf) = 0;
  void Compile_srlv_const(CompileFlags cf);
  virtual void Compile_srlv(CompileFlags cf) = 0;
  void Compile_srav_const(CompileFlags cf);
  virtual void Compile_srav(CompileFlags cf) = 0;
  void Compile_mult_const(CompileFlags cf);
  virtual void Compile_mult(CompileFlags cf) = 0;
  void Compile_multu_const(CompileFlags cf);
  virtual void Compile_multu(CompileFlags cf) = 0;
  void Compile_div_const(CompileFlags cf);
  virtual void Compile_div(CompileFlags cf) = 0;
  void Compile_divu_const(CompileFlags cf);
  virtual void Compile_divu(CompileFlags cf) = 0;
  void Compile_add_const(CompileFlags cf);
  virtual void Compile_add(CompileFlags cf) = 0;
  void Compile_addu_const(CompileFlags cf);
  virtual void Compile_addu(CompileFlags cf) = 0;
  void Compile_sub_const(CompileFlags cf);
  virtual void Compile_sub(CompileFlags cf) = 0;
  void Compile_subu_const(CompileFlags cf);
  virtual void Compile_subu(CompileFlags cf) = 0;
  void Compile_and_const(CompileFlags cf);
  virtual void Compile_and(CompileFlags cf) = 0;
  void Compile_or_const(CompileFlags cf);
  virtual void Compile_or(CompileFlags cf) = 0;
  void Compile_xor_const(CompileFlags cf);
  virtual void Compile_xor(CompileFlags cf) = 0;
  void Compile_nor_const(CompileFlags cf);
  virtual void Compile_nor(CompileFlags cf) = 0;
  void Compile_slt_const(CompileFlags cf);
  virtual void Compile_slt(CompileFlags cf) = 0;
  void Compile_sltu_const(CompileFlags cf);
  virtual void Compile_sltu(CompileFlags cf) = 0;

  void Compile_addi_const(CompileFlags cf);
  virtual void Compile_addi(CompileFlags cf) = 0;
  void Compile_addiu_const(CompileFlags cf);
  virtual void Compile_addiu(CompileFlags cf) = 0;
  void Compile_slti_const(CompileFlags cf);
  virtual void Compile_slti(CompileFlags cf) = 0;
  void Compile_sltiu_const(CompileFlags cf);
  virtual void Compile_sltiu(CompileFlags cf) = 0;
  void Compile_andi_const(CompileFlags cf);
  virtual void Compile_andi(CompileFlags cf) = 0;
  void Compile_ori_const(CompileFlags cf);
  virtual void Compile_ori(CompileFlags cf) = 0;
  void Compile_xori_const(CompileFlags cf);
  virtual void Compile_xori(CompileFlags cf) = 0;
  void Compile_lui();

  virtual void Compile_Load(CompileFlags cf, MemoryAccessSize size, bool sign,
                            const std::optional<VirtualMemoryAddress>& address) = 0;
  virtual void Compile_Store(CompileFlags cf, MemoryAccessSize size,
                             const std::optional<VirtualMemoryAddress>& address) = 0;

  static u32* GetCop0RegPtr(Cop0Reg reg);
  static u32 GetCop0RegWriteMask(Cop0Reg reg);

  void Compile_mfc0(CompileFlags cf);
  virtual void Compile_mtc0(CompileFlags cf) = 0;
  virtual void Compile_rfe(CompileFlags cf) = 0;

  void AddGTETicks(TickCount ticks);
  void StallUntilGTEComplete();
  virtual void Compile_mfc2(CompileFlags cf) = 0;
  virtual void Compile_mtc2(CompileFlags cf) = 0;
  virtual void Compile_cop2(CompileFlags cf) = 0;

  enum GTERegisterAccessAction : u8
  {
    Ignore,
    Direct,
    ZeroExtend16,
    SignExtend16,
    CallHandler,
    PushFIFO,
  };

  static std::pair<u32*, GTERegisterAccessAction> GetGTERegisterPointer(u32 index, bool writing);

  Block* m_block = nullptr;
  u32 m_compiler_pc = 0;
  TickCount m_cycles = 0;

  const Instruction* inst = nullptr;
  u32 m_current_instruction_pc = 0;
  bool m_current_instruction_branch_delay_slot = false;

  bool m_dirty_pc = false;
  bool m_dirty_instruction_bits = false;
  bool m_block_ended = false;

  std::bitset<static_cast<size_t>(Reg::count)> m_constant_regs_valid = {};
  std::bitset<static_cast<size_t>(Reg::count)> m_constant_regs_dirty = {};
  std::array<u32, static_cast<size_t>(Reg::count)> m_constant_reg_values = {};

  std::array<HostRegAlloc, NUM_HOST_REGS> m_host_regs = {};
  u16 m_register_alloc_counter = 0;

  bool m_load_delay_dirty = true;
  Reg m_load_delay_register = Reg::count;
  u32 m_load_delay_value_register = 0;

  Reg m_next_load_delay_register = Reg::count;
  u32 m_next_load_delay_value_register = 0;

  struct HostStateBackup
  {
    TickCount cycles;
    u32 compiler_pc;
    bool dirty_pc;
    bool dirty_instruction_bits;
    bool block_ended;
    const Instruction* inst;
    u32 current_instruction_pc;
    bool current_instruction_delay_slot;
    std::bitset<static_cast<size_t>(Reg::count)> const_regs_valid;
    std::bitset<static_cast<size_t>(Reg::count)> const_regs_dirty;
    std::array<u32, static_cast<size_t>(Reg::count)> const_regs_values;
    std::array<HostRegAlloc, NUM_HOST_REGS> host_regs;
    u16 register_alloc_counter;
    bool load_delay_dirty;
    Reg load_delay_register;
    u32 load_delay_value_register;
    Reg next_load_delay_register;
    u32 next_load_delay_value_register;
  };

  // we need two of these, one for branch delays, and another if we have an overflow in the delay slot
  std::array<HostStateBackup, 2> m_host_state_backup = {};
  u32 m_host_state_backup_count = 0;
};
} // namespace CPU::NewRec
