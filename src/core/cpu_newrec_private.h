// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "bus.h"
#include "cpu_core_private.h"
#include "cpu_types.h"
#include "types.h"
#include "util/jit_code_buffer.h"

namespace CPU::NewRec {
enum : u32
{
  LUT_TABLE_COUNT = 0x10000,
  LUT_TABLE_SIZE = 0x10000 / sizeof(u32), // 16384, one for each PC
  LUT_TABLE_SHIFT = 16,
};

struct Block
{
  u32 pc;
  u32 size; // in guest instructions
  const void* host_code;

  // links to previous/next block within page
  Block* next_block_in_page;

  bool invalidated;

  // followed by Instruction * size
  const Instruction* Instructions() const { return reinterpret_cast<const Instruction*>(this + 1); }
  Instruction* Instructions() { return reinterpret_cast<Instruction*>(this + 1); }
};

using CodeLUT = const void**;
using CodeLUTArray = std::array<CodeLUT, LUT_TABLE_COUNT>;
using BlockLUTArray = std::array<Block**, LUT_TABLE_COUNT>;

static constexpr bool BlockInRAM(VirtualMemoryAddress pc)
{
  return VirtualAddressToPhysical(pc) < Bus::g_ram_size;
}

Block* LookupBlock(u32 pc);
Block* CreateBlock(u32 pc);
bool RevalidateBlock(Block* block);

void CompileASMFunctions();

void SetFastMap(u32 pc, const void* function);

extern JitCodeBuffer g_code_buffer;

extern CodeLUTArray g_fast_map;

extern const void* g_enter_recompiler;
extern const void* g_exit_recompiler;
extern const void* g_compile_block;
extern const void* g_event_test_and_dispatch;
extern const void* g_dispatcher;
extern const void* g_interpret_block;

} // namespace CPU::NewRec
