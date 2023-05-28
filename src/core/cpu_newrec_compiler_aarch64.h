// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "cpu_newrec_compiler.h"
#include <memory>

#include "vixl/aarch64/assembler-aarch64.h"

namespace CPU::NewRec {

class AArch64Compiler final : public Compiler
{
public:
  AArch64Compiler();
  ~AArch64Compiler() override;

protected:
  void DisassembleAndLog(const void* start, u32 size) override;
  u32 GetHostInstructionCount(const void* start, u32 size) override;
  const char* GetHostRegName(u32 reg) const override;

  const void* GetCurrentCodePointer() override;

  void LoadHostRegWithConstant(u32 reg, u32 val) override;
  void LoadHostRegFromCPUPointer(u32 reg, const void* ptr) override;
  void StoreConstantToCPUPointer(u32 val, const void* ptr) override;
  void StoreHostRegToCPUPointer(u32 reg, const void* ptr) override;
  void CopyHostReg(u32 dst, u32 src) override;

  void Reset(Block* block, u8* code_buffer, u32 code_buffer_space, u8* far_code_buffer, u32 far_code_space) override;
  void BeginBlock() override;
  void EndBlock(const std::optional<u32>& newpc) override;
  void EndBlockWithException(Exception excode) override;
  void EndAndLinkBlock(const std::optional<u32>& newpc);
  const void* EndCompile(u32* code_size, u32* far_code_size) override;

  void Flush(u32 flags) override;

  void Compile_Fallback() override;

  void CheckBranchTarget(const vixl::aarch64::WRegister& pcreg);
  void Compile_jr(CompileFlags cf) override;
  void Compile_jalr(CompileFlags cf) override;
  void Compile_bxx(CompileFlags cf, BranchCondition cond) override;

  void Compile_addi(CompileFlags cf, bool overflow);
  void Compile_addi(CompileFlags cf) override;
  void Compile_addiu(CompileFlags cf) override;
  void Compile_slti(CompileFlags cf, bool sign);
  void Compile_slti(CompileFlags cf) override;
  void Compile_sltiu(CompileFlags cf) override;
  void Compile_andi(CompileFlags cf) override;
  void Compile_ori(CompileFlags cf) override;
  void Compile_xori(CompileFlags cf) override;

  void Compile_shift(CompileFlags cf, void (vixl::aarch64::Assembler::*op)(const vixl::aarch64::Register&,
                                                                           const vixl::aarch64::Register&, unsigned));
  void Compile_sll(CompileFlags cf) override;
  void Compile_srl(CompileFlags cf) override;
  void Compile_sra(CompileFlags cf) override;
  void Compile_variable_shift(CompileFlags cf,
                              void (vixl::aarch64::Assembler::*op)(const vixl::aarch64::Register&,
                                                                   const vixl::aarch64::Register&,
                                                                   const vixl::aarch64::Register&),
                              void (vixl::aarch64::Assembler::*op_const)(const vixl::aarch64::Register&,
                                                                         const vixl::aarch64::Register&, unsigned));
  void Compile_sllv(CompileFlags cf) override;
  void Compile_srlv(CompileFlags cf) override;
  void Compile_srav(CompileFlags cf) override;
  void Compile_mult(CompileFlags cf, bool sign);
  void Compile_mult(CompileFlags cf) override;
  void Compile_multu(CompileFlags cf) override;
  void Compile_div(CompileFlags cf) override;
  void Compile_divu(CompileFlags cf) override;
  void TestOverflow(const vixl::aarch64::WRegister& result);
  void Compile_dst_op(CompileFlags cf,
                      void (vixl::aarch64::Assembler::*op)(const vixl::aarch64::Register&,
                                                           const vixl::aarch64::Register&,
                                                           const vixl::aarch64::Operand&),
                      bool commutative, bool logical, bool overflow);
  void Compile_add(CompileFlags cf) override;
  void Compile_addu(CompileFlags cf) override;
  void Compile_sub(CompileFlags cf) override;
  void Compile_subu(CompileFlags cf) override;
  void Compile_and(CompileFlags cf) override;
  void Compile_or(CompileFlags cf) override;
  void Compile_xor(CompileFlags cf) override;
  void Compile_nor(CompileFlags cf) override;
  void Compile_slt(CompileFlags cf, bool sign);
  void Compile_slt(CompileFlags cf) override;
  void Compile_sltu(CompileFlags cf) override;

  void FlushForLoadStore(const std::optional<VirtualMemoryAddress>& address, bool store);
  vixl::aarch64::WRegister
  ComputeLoadStoreAddressArg(CompileFlags cf, const std::optional<VirtualMemoryAddress>& address,
                             const std::optional<const vixl::aarch64::WRegister>& reg = std::nullopt);
  template<typename RegAllocFn>
  void GenerateLoad(const vixl::aarch64::WRegister& addr_reg, MemoryAccessSize size, bool sign,
                    const RegAllocFn& dst_reg_alloc);
  void GenerateStore(const vixl::aarch64::WRegister& addr_reg, const vixl::aarch64::WRegister& value_reg,
                     MemoryAccessSize size);
  void Compile_lxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                   const std::optional<VirtualMemoryAddress>& address) override;
  void Compile_lwx(CompileFlags cf, MemoryAccessSize size, bool sign,
                   const std::optional<VirtualMemoryAddress>& address) override;
  void Compile_lwc2(CompileFlags cf, MemoryAccessSize size, bool sign,
                    const std::optional<VirtualMemoryAddress>& address) override;
  void Compile_sxx(CompileFlags cf, MemoryAccessSize size, bool sign,
                   const std::optional<VirtualMemoryAddress>& address) override;
  void Compile_swx(CompileFlags cf, MemoryAccessSize size, bool sign,
                   const std::optional<VirtualMemoryAddress>& address) override;
  void Compile_swc2(CompileFlags cf, MemoryAccessSize size, bool sign,
                    const std::optional<VirtualMemoryAddress>& address) override;

  void TestInterrupts(const vixl::aarch64::WRegister& sr);
  void Compile_mtc0(CompileFlags cf) override;
  void Compile_rfe(CompileFlags cf) override;

  void Compile_mfc2(CompileFlags cf) override;
  void Compile_mtc2(CompileFlags cf) override;
  void Compile_cop2(CompileFlags cf) override;

private:
  void EmitMov(const vixl::aarch64::WRegister& dst, u32 val);
  void EmitCall(const void* ptr, bool force_inline = false);

  vixl::aarch64::Operand armCheckAddSubConstant(s32 val);
  vixl::aarch64::Operand armCheckAddSubConstant(u32 val);
  vixl::aarch64::Operand armCheckCompareConstant(s32 val);
  vixl::aarch64::Operand armCheckLogicalConstant(u32 val);

  void SwitchToFarCode(bool emit_jump, vixl::aarch64::Condition cond = vixl::aarch64::Condition::al);
  void SwitchToFarCodeIfBitSet(const vixl::aarch64::Register& reg, u32 bit);
  void SwitchToFarCodeIfRegZeroOrNonZero(const vixl::aarch64::Register& reg, bool nonzero);
  void SwitchToNearCode(bool emit_jump, vixl::aarch64::Condition cond = vixl::aarch64::Condition::al);

  void AssertRegOrConstS(CompileFlags cf) const;
  void AssertRegOrConstT(CompileFlags cf) const;
  vixl::aarch64::MemOperand MipsPtr(Reg r) const;
  vixl::aarch64::WRegister CFGetRegD(CompileFlags cf) const;
  vixl::aarch64::WRegister CFGetRegS(CompileFlags cf) const;
  vixl::aarch64::WRegister CFGetRegT(CompileFlags cf) const;
  vixl::aarch64::WRegister CFGetRegLO(CompileFlags cf) const;
  vixl::aarch64::WRegister CFGetRegHI(CompileFlags cf) const;

  void MoveSToReg(const vixl::aarch64::WRegister& dst, CompileFlags cf);
  void MoveTToReg(const vixl::aarch64::WRegister& dst, CompileFlags cf);

  std::unique_ptr<vixl::aarch64::Assembler> m_emitter;
  std::unique_ptr<vixl::aarch64::Assembler> m_far_emitter;
  vixl::aarch64::Assembler* armAsm;

#ifdef VIXL_DEBUG
  std::unique_ptr<vixl::CodeBufferCheckScope> m_emitter_check;
  std::unique_ptr<vixl::CodeBufferCheckScope> m_far_emitter_check;
#endif
};

} // namespace CPU::NewRec
