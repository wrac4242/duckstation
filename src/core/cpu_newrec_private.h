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

enum RegInfoFlags : u8
{
  RI_LIVE = (1 << 0),
  RI_USED = (1 << 1),
  RI_LASTUSE = (1 << 2),
};

struct InstructionInfo
{
  u8 reg_flags[static_cast<u8>(Reg::count)];
  // Reg write_reg[3];
  Reg read_reg[3];

  // If unset, values which are not live will not be written back to memory.
  // Tends to break stuff at the moment.
  static constexpr bool WRITE_DEAD_VALUES = true;

  /// Returns true if the register is used later in the block, and this isn't the last instruction to use it.
  /// In other words, the register is worth keeping in a host register/caching it.
  inline bool UsedTest(Reg reg) const { return (reg_flags[static_cast<u8>(reg)] & (RI_USED | RI_LASTUSE)) == RI_USED; }

  /// Returns true if the value should be computed/written back.
  /// Basically, this means it's either used before it's overwritten, or not overwritten by the end of the block.
  inline bool LiveTest(Reg reg) const
  {
    return WRITE_DEAD_VALUES || ((reg_flags[static_cast<u8>(reg)] & RI_LIVE) != 0);
  }

  /// Returns true if the register can be renamed into another.
  inline bool RenameTest(Reg reg) const { return (reg == Reg::zero || !UsedTest(reg) || !LiveTest(reg)); }

  /// Returns true if this instruction reads this register.
  inline bool ReadsReg(Reg reg) const { return (read_reg[0] == reg || read_reg[1] == reg || read_reg[2] == reg); }
};

enum class BlockState : u8
{
  Valid,
  Invalidated,
  NeedsRecompile,
};

struct Block
{
  u32 pc;
  u32 size; // in guest instructions
  const void* host_code;

  // links to previous/next block within page
  Block* next_block_in_page;

  BlockLinkMap::iterator exit_links[MAX_BLOCK_EXIT_LINKS];
  u32 num_exit_links;

  BlockState state;

  // followed by Instruction * size, InstructionRegInfo * size
  const Instruction* Instructions() const { return reinterpret_cast<const Instruction*>(this + 1); }
  Instruction* Instructions() { return reinterpret_cast<Instruction*>(this + 1); }

  const InstructionInfo* InstructionsInfo() const
  {
    return reinterpret_cast<const InstructionInfo*>(Instructions() + size);
  }
  InstructionInfo* InstructionsInfo() { return reinterpret_cast<InstructionInfo*>(Instructions() + size); }
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
  u8 code_size;
  u8 fault_count;

  MemoryAccessSize AccessSize() const { return static_cast<MemoryAccessSize>(size); }
  u32 AccessSizeInBytes() const { return 1u << size; }
};
static_assert(sizeof(LoadstoreBackpatchInfo) == 16);

static inline bool BlockInRAM(VirtualMemoryAddress pc)
{
  return VirtualAddressToPhysical(pc) < Bus::g_ram_size;
}

enum class PageProtectionMode : u8
{
  WriteProtected,
  ManualCheck,
  Unprotected,
};
struct PageProtectionInfo
{
  Block* first_block_in_page;
  Block* last_block_in_page;

  PageProtectionMode mode;
  u16 invalidate_count;
  u32 invalidate_frame;
};
static_assert(sizeof(PageProtectionInfo) == 24);

Block* LookupBlock(u32 pc);
Block* CreateBlock(u32 pc);
bool RevalidateBlock(Block* block);
void CompileOrRevalidateBlock(u32 start_pc);
void DiscardAndRecompileBlock(u32 start_pc);
u32 CreateBlockLink(Block* from_block, void* code, u32 newpc);
PageProtectionMode GetProtectionModeForBlock(Block* block);

u32 CompileASMFunctions(u8* code, u32 code_size);
u32 EmitJump(void* code, const void* dst, bool flush_icache);

void SetFastMap(u32 pc, const void* function);

void AddLoadStoreInfo(void* code_address, u32 code_size, u32 guest_pc, TickCount cycles, u32 gpr_bitmask,
                      u8 address_register, u8 data_register, MemoryAccessSize size, bool is_signed, bool is_load);
u32 BackpatchLoadStore(void* thunk_code, u32 thunk_space, void* code_address, u32 code_size, TickCount cycles_to_add,
                       TickCount cycles_to_remove, u32 gpr_bitmask, u8 address_register, u8 data_register,
                       MemoryAccessSize size, bool is_signed, bool is_load);

extern CodeLUTArray g_fast_map;

extern NORETURN_FUNCTION_POINTER void(*g_enter_recompiler)();
extern const void* g_compile_or_revalidate_block;
extern const void* g_check_events_and_dispatch;
extern const void* g_dispatcher;
extern const void* g_interpret_block;
extern const void* g_discard_and_recompile_block;

} // namespace CPU::NewRec
