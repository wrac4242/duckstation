// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "cpu_newrec_compiler.h"
#include <memory>

// We need to include windows.h before xbyak does..
#ifdef _WIN32
#include "common/windows_headers.h"
#endif

#define XBYAK_NO_OP_NAMES 1
#include "xbyak.h"

namespace CPU::NewRec {

class X64Compiler final : public Compiler
{
public:
  X64Compiler();
  ~X64Compiler() override;

  static u32 CompileASMFunctions(u8* code, u32 code_size);

protected:
  void DisassembleAndLog(const void* start, u32 size) override;
  u32 GetHostInstructionCount(const void* start, u32 size) override;

  const void* GetCurrentCodePointer() override;

  void LoadHostRegWithConstant(u32 reg, u32 val);
  void LoadHostRegFromCPUPointer(u32 reg, const void* ptr);
  void StoreConstantToCPUPointer(u32 val, const void* ptr);
  void StoreHostRegToCPUPointer(u32 reg, const void* ptr);
  void CopyHostReg(u32 dst, u32 src) override;

  void Reset(Block* block, u8* code_buffer, u32 code_buffer_space, u8* far_code_buffer, u32 far_code_space) override;
  void BeginBlock() override;
  void EndBlock(const std::optional<u32>& newpc) override;
  void EndBlockWithException(Exception excode) override;
  void EndAndLinkBlock(const std::optional<u32>& newpc);
  const void* EndCompile(u32* code_size, u32* far_code_size) override;

  void Flush(u32 flags) override;

  void Compile_Fallback() override;

  void CheckBranchTarget(const Xbyak::Reg32& pcreg);
  void Compile_jr(CompileFlags cf) override;
  void Compile_jalr(CompileFlags cf) override;
  void Compile_bxx(CompileFlags cf, BranchCondition cond) override;

  void Compile_addi(CompileFlags cf) override;
  void Compile_addiu(CompileFlags cf) override;
  void Compile_slti(CompileFlags cf, bool sign);
  void Compile_slti(CompileFlags cf) override;
  void Compile_sltiu(CompileFlags cf) override;
  void Compile_andi(CompileFlags cf) override;
  void Compile_ori(CompileFlags cf) override;
  void Compile_xori(CompileFlags cf) override;

  void Compile_sll(CompileFlags cf) override;
  void Compile_srl(CompileFlags cf) override;
  void Compile_sra(CompileFlags cf) override;
  void Compile_variable_shift(CompileFlags cf,
                              void (Xbyak::CodeGenerator::*op)(const Xbyak::Operand&, const Xbyak::Reg8&),
                              void (Xbyak::CodeGenerator::*op_const)(const Xbyak::Operand&, int));
  void Compile_sllv(CompileFlags cf) override;
  void Compile_srlv(CompileFlags cf) override;
  void Compile_srav(CompileFlags cf) override;
  void Compile_mult(CompileFlags cf, bool sign);
  void Compile_mult(CompileFlags cf) override;
  void Compile_multu(CompileFlags cf) override;
  void Compile_div(CompileFlags cf) override;
  void Compile_divu(CompileFlags cf) override;
  void TestOverflow(const Xbyak::Reg32& result);
  void Compile_dst_op(CompileFlags cf, void (Xbyak::CodeGenerator::*op)(const Xbyak::Operand&, const Xbyak::Operand&),
                      void (Xbyak::CodeGenerator::*op_const)(const Xbyak::Operand&, u32), bool commutative,
                      bool overflow);
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
  Xbyak::Reg32 ComputeLoadStoreAddressArg(CompileFlags cf, const std::optional<VirtualMemoryAddress>& address,
                                          const std::optional<const Xbyak::Reg32>& reg = std::nullopt);
  template<typename RegAllocFn>
  void GenerateLoad(const Xbyak::Reg32& addr_reg, MemoryAccessSize size, bool sign, const RegAllocFn& dst_reg_alloc);
  void GenerateStore(const Xbyak::Reg32& addr_reg, const Xbyak::Reg32& value_reg, MemoryAccessSize size);
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

  void TestInterrupts(const Xbyak::Reg32& sr);
  void Compile_mtc0(CompileFlags cf) override;
  void Compile_rfe(CompileFlags cf) override;

  void Compile_mfc2(CompileFlags cf) override;
  void Compile_mtc2(CompileFlags cf) override;
  void Compile_cop2(CompileFlags cf) override;

private:
  static bool IsCallerSavedRegister(u32 id);
  void SwitchToFarCode(bool emit_jump, void (Xbyak::CodeGenerator::*jump_op)(const void*) = nullptr);
  void SwitchToNearCode(bool emit_jump, void (Xbyak::CodeGenerator::*jump_op)(const void*) = nullptr);

  Xbyak::Address MipsPtr(Reg r) const;
  Xbyak::Reg32 CFGetRegD(CompileFlags cf) const;
  Xbyak::Reg32 CFGetRegS(CompileFlags cf) const;
  Xbyak::Reg32 CFGetRegT(CompileFlags cf) const;
  Xbyak::Reg32 CFGetRegLO(CompileFlags cf) const;
  Xbyak::Reg32 CFGetRegHI(CompileFlags cf) const;

  Xbyak::Reg32 MoveSToD(CompileFlags cf);
  Xbyak::Reg32 MoveSToT(CompileFlags cf);
  Xbyak::Reg32 MoveTToD(CompileFlags cf);
  void MoveSToReg(const Xbyak::Reg32& dst, CompileFlags cf);
  void MoveTToReg(const Xbyak::Reg32& dst, CompileFlags cf);

  std::unique_ptr<Xbyak::CodeGenerator> m_emitter;
  std::unique_ptr<Xbyak::CodeGenerator> m_far_emitter;
  Xbyak::CodeGenerator* cg;
};

} // namespace CPU::NewRec
