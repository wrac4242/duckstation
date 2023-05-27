// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#pragma once
#include "bus.h"
#include "cpu_core_private.h"
#include "cpu_newrec.h"
#include "cpu_types.h"
#include "types.h"
#include "util/jit_code_buffer.h"
#include <unordered_map>

namespace CPU::NewRec {
enum : u32
{
  LUT_TABLE_COUNT = 0x10000,
  LUT_TABLE_SIZE = 0x10000 / sizeof(u32), // 16384, one for each PC
  LUT_TABLE_SHIFT = 16,

  MAX_BLOCK_EXIT_LINKS = 2,
};

using CodeLUT = const void**;
using CodeLUTArray = std::array<CodeLUT, LUT_TABLE_COUNT>;
using BlockLinkMap = std::unordered_multimap<u32, void*>; // TODO: try ordered?

struct Block
{
  u32 pc;
  u32 size; // in guest instructions
  const void* host_code;

  // links to previous/next block within page
  Block* next_block_in_page;

  BlockLinkMap::iterator exit_links[MAX_BLOCK_EXIT_LINKS];
  u32 num_exit_links;

  bool invalidated;

  // followed by Instruction * size
  const Instruction* Instructions() const { return reinterpret_cast<const Instruction*>(this + 1); }
  Instruction* Instructions() { return reinterpret_cast<Instruction*>(this + 1); }
};

using BlockLUTArray = std::array<Block**, LUT_TABLE_COUNT>;

struct LoadstoreBackpatchInfo
{
  u32 guest_pc;
  u32 gpr_bitmask;
  u16 cycles;
  u16 address_register : 5;
  u16 data_register : 5;
  u16 size : 2;
  u16 is_signed : 1;
  u16 is_load : 1;
  u16 code_size;

  MemoryAccessSize AccessSize() const { return static_cast<MemoryAccessSize>(size); }
  u32 AccessSizeInBytes() const { return 1u << size; }
};
static_assert(sizeof(LoadstoreBackpatchInfo) == 16);

static inline bool BlockInRAM(VirtualMemoryAddress pc)
{
  return VirtualAddressToPhysical(pc) < Bus::g_ram_size;
}

Block* LookupBlock(u32 pc);
Block* CreateBlock(u32 pc);
bool RevalidateBlock(Block* block);
void CompileOrRevalidateBlock(u32 start_pc);
u32 CreateBlockLink(Block* from_block, void* code, u32 newpc);

u32 CompileASMFunctions(u8* code, u32 code_size);
u32 EmitJump(void* code, const void* dst, bool flush_icache);

void SetFastMap(u32 pc, const void* function);

void AddLoadStoreInfo(void* code_address, u32 code_size, u32 guest_pc, TickCount cycles, u32 gpr_bitmask,
                      u8 address_register, u8 data_register, MemoryAccessSize size, bool is_signed, bool is_load);
bool BackpatchLoadStore(void* code_address, u32 guest_address);
u32 BackpatchLoadStore(void* thunk_code, u32 thunk_space, void* code_address, u32 code_size, TickCount cycles_to_add,
                       TickCount cycles_to_remove, u32 gpr_bitmask, u8 address_register, u8 data_register,
                       MemoryAccessSize size, bool is_signed, bool is_load);

extern CodeLUTArray g_fast_map;

extern const void* g_enter_recompiler;
extern const void* g_exit_recompiler;
extern const void* g_compile_or_revalidate_block;
extern const void* g_check_events_and_dispatch;
extern const void* g_dispatcher;
extern const void* g_interpret_block;

} // namespace CPU::NewRec
