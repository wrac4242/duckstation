#include "cpu_newrec.h"
#include "bus.h"
#include "common/align.h"
#include "common/assert.h"
#include "common/log.h"
#include "cpu_code_cache.h"
#include "cpu_core_private.h"
#include "cpu_newrec_compiler.h"
#include "cpu_newrec_private.h"
#include "cpu_types.h"
#include "settings.h"
#include <unordered_map>
#include <vector>
Log_SetChannel(CPU::NewRec);

namespace CPU::NewRec {
using LUTRangeList = std::array<std::pair<VirtualMemoryAddress, VirtualMemoryAddress>, 9>;
using PageBlockLookupArray = std::array<std::pair<Block*, Block*>, Bus::RAM_8MB_CODE_PAGE_COUNT>;

static CodeLUT DecodeCodeLUTPointer(u32 slot, CodeLUT ptr);
static CodeLUT EncodeCodeLUTPointer(u32 slot, CodeLUT ptr);
static CodeLUT OffsetCodeLUTPointer(CodeLUT fake_ptr, u32 pc);

static void InvalidCodeFunction();

static void AllocateLUTs();
static void ResetLUTs();

static u32 ReadBlockInstructions(u32 start_pc);
static void AddBlockToPageList(Block* block);
static void BacklinkBlocks(u32 pc, const void* dst);
static void UnlinkBlockExits(Block* block);

// Fast map provides lookup from PC to function
// Function pointers are offset so that you don't need to subtract
CodeLUTArray g_fast_map;
static BlockLUTArray s_block_map;
static std::unique_ptr<const void*[]> s_lut_code_pointers;
static std::unique_ptr<Block*[]> s_lut_block_pointers;
static PageBlockLookupArray s_page_block_lookup;
static std::vector<Block*> s_blocks;
static BlockLinkMap s_block_links;
static bool s_lut_initialized = false;

// for compiling
static std::vector<Instruction> s_block_instructions;

JitCodeBuffer g_code_buffer;

const void* g_enter_recompiler;
const void* g_exit_recompiler;
const void* g_compile_block;
const void* g_check_events_and_dispatch;
const void* g_dispatcher;

static constexpr u32 RECOMPILER_GUARD_SIZE = 4096;
alignas(4096) static u8 s_code_storage[64 * 1024 * 1024];
} // namespace CPU::NewRec

namespace CPU::NewRec {
static constexpr u32 GetLUTTableCount(u32 start, u32 end)
{
  return ((end >> LUT_TABLE_SHIFT) - (start >> LUT_TABLE_SHIFT)) + 1;
}

static constexpr CPU::NewRec::LUTRangeList GetLUTRanges()
{
  const CPU::NewRec::LUTRangeList ranges = {{
    {0x00000000, 0x00800000}, // RAM
    {0x1F000000, 0x1F800000}, // EXP1
    {0x1FC00000, 0x1FC80000}, // BIOS

    {0x80000000, 0x80800000}, // RAM
    {0x9F000000, 0x9F800000}, // EXP1
    {0x9FC00000, 0x9FC80000}, // BIOS

    {0xA0000000, 0xA0800000}, // RAM
    {0xBF000000, 0xBF800000}, // EXP1
    {0xBFC00000, 0xBFC80000}  // BIOS
  }};
  return ranges;
}

static constexpr u32 GetLUTSlotCount(bool include_unreachable)
{
  u32 tables = include_unreachable ? 1 : 0; // unreachable table
  for (const auto [start, end] : GetLUTRanges())
    tables += GetLUTTableCount(start, end);

  return tables * LUT_TABLE_SIZE;
}
} // namespace CPU::NewRec

CPU::NewRec::CodeLUT CPU::NewRec::DecodeCodeLUTPointer(u32 slot, CodeLUT ptr)
{
  if constexpr (sizeof(void*) == 8)
    return reinterpret_cast<CodeLUT>(reinterpret_cast<u8*>(ptr) + (static_cast<u64>(slot) << 17));
  else
    return reinterpret_cast<CodeLUT>(reinterpret_cast<u8*>(ptr) + (slot << 16));
}

CPU::NewRec::CodeLUT CPU::NewRec::EncodeCodeLUTPointer(u32 slot, CodeLUT ptr)
{
  if constexpr (sizeof(void*) == 8)
    return reinterpret_cast<CodeLUT>(reinterpret_cast<u8*>(ptr) - (static_cast<u64>(slot) << 17));
  else
    return reinterpret_cast<CodeLUT>(reinterpret_cast<u8*>(ptr) - (slot << 16));
}

CPU::NewRec::CodeLUT CPU::NewRec::OffsetCodeLUTPointer(CodeLUT fake_ptr, u32 pc)
{
  u8* fake_byte_ptr = reinterpret_cast<u8*>(fake_ptr);
  if constexpr (sizeof(void*) == 8)
    return reinterpret_cast<const void**>(fake_byte_ptr + (static_cast<u64>(pc) << 1));
  else
    return reinterpret_cast<const void**>(fake_byte_ptr + pc);
}

void CPU::NewRec::AllocateLUTs()
{
  constexpr u32 num_code_slots = GetLUTSlotCount(true);
  constexpr u32 num_block_slots = GetLUTSlotCount(false);

  Assert(!s_lut_code_pointers && !s_lut_block_pointers);
  s_lut_code_pointers = std::make_unique<const void*[]>(num_code_slots);
  s_lut_block_pointers = std::make_unique<Block*[]>(num_block_slots);
  std::memset(s_lut_block_pointers.get(), 0, sizeof(Block*) * num_block_slots);

  CodeLUT code_table_ptr = s_lut_code_pointers.get();
  Block** block_table_ptr = s_lut_block_pointers.get();
  CodeLUT const code_table_ptr_end = code_table_ptr + num_code_slots;
  Block** const block_table_ptr_end = block_table_ptr + num_block_slots;

  // Make the unreachable table jump to the invalid code callback.
  for (u32 i = 0; i < LUT_TABLE_COUNT; i++)
    code_table_ptr[i] = InvalidCodeFunction;

  // Mark everything as unreachable to begin with.
  for (u32 i = 0; i < LUT_TABLE_COUNT; i++)
  {
    g_fast_map[i] = EncodeCodeLUTPointer(i, code_table_ptr);
    s_block_map[i] = nullptr;
  }
  code_table_ptr += LUT_TABLE_SIZE;

  // Allocate ranges.
  for (const auto [start, end] : GetLUTRanges())
  {
    const u32 start_slot = start >> LUT_TABLE_SHIFT;
    const u32 count = GetLUTTableCount(start, end);
    for (u32 i = 0; i < count; i++)
    {
      const u32 slot = start_slot + i;

      g_fast_map[slot] = EncodeCodeLUTPointer(slot, code_table_ptr);
      code_table_ptr += LUT_TABLE_SIZE;

      s_block_map[slot] = block_table_ptr;
      block_table_ptr += LUT_TABLE_SIZE;
    }
  }

  Assert(code_table_ptr == code_table_ptr_end);
  Assert(block_table_ptr == block_table_ptr_end);
}

void CPU::NewRec::ResetLUTs()
{
  if (!s_lut_code_pointers)
    return;

  for (u32 i = 0; i < LUT_TABLE_COUNT; i++)
  {
    CodeLUT ptr = DecodeCodeLUTPointer(i, g_fast_map[i]);
    if (ptr == s_lut_code_pointers.get())
      continue;

    for (u32 j = 0; j < LUT_TABLE_SIZE; j++)
      ptr[j] = g_compile_block;
  }
}

void CPU::NewRec::SetFastMap(u32 pc, const void* function)
{
  if (!s_lut_code_pointers)
    return;

  const u32 table = pc >> LUT_TABLE_SHIFT;
  CodeLUT encoded_ptr = g_fast_map[table];

#ifdef _DEBUG
  const CodeLUT table_ptr = DecodeCodeLUTPointer(table, encoded_ptr);
  DebugAssert(table_ptr != nullptr && table_ptr != s_lut_code_pointers.get());
#endif

  *OffsetCodeLUTPointer(encoded_ptr, pc) = function;
}

CPU::NewRec::Block* CPU::NewRec::LookupBlock(u32 pc)
{
  const u32 table = pc >> LUT_TABLE_SHIFT;
  if (!s_block_map[table])
    return nullptr;

  const u32 idx = (pc & 0xFFFF) >> 2;
  return s_block_map[table][idx];
}

u32 CPU::NewRec::ReadBlockInstructions(u32 start_pc)
{
  u32 pc = start_pc;
  bool is_branch = false;
  bool is_branch_delay = false;

  // TODO: Jump to other block if it exists at this pc?

  s_block_instructions.clear();

  for (;;)
  {
    Instruction i;
    if (!SafeReadInstruction(pc, &i.bits) || !IsInvalidInstruction(i))
      break;

    is_branch_delay = is_branch;
    is_branch = IsBranchInstruction(i);
    s_block_instructions.push_back(i);
    pc += sizeof(Instruction);

    if (is_branch_delay)
      break;

    if (IsExitBlockInstruction(i))
      break;
  }

  return static_cast<u32>(s_block_instructions.size());
}

CPU::NewRec::Block* CPU::NewRec::CreateBlock(u32 pc)
{
  const u32 size = ReadBlockInstructions(pc);
  if (size == 0)
  {
    Log_ErrorPrintf("Cannot compile block at pc %08X", pc);
    return nullptr;
  }

  const u32 table = pc >> LUT_TABLE_SHIFT;
  Assert(s_block_map[table]);

  const u32 idx = (pc & 0xFFFF) >> 2;
  Block* block = s_block_map[table][idx];
  if (block)
  {
    // shouldn't be in the page list.. since we should come here after invalidating
    Assert(!block->next_block_in_page);

    // if it has the same number of instructions, we can reuse it
    if (block->size != size)
    {
      // this sucks.. hopefully won't happen very often
      // TODO: allocate max size, allow shrink but not grow
      auto it = std::find(s_blocks.begin(), s_blocks.end(), block);
      Assert(it != s_blocks.end());
      s_blocks.erase(it);

      std::free(block);
      block = nullptr;
    }
  }

  if (!block)
  {
    block = static_cast<Block*>(std::malloc(sizeof(Block) + (sizeof(Instruction) * size)));
    Assert(block);
    s_blocks.push_back(block);
  }

  block->pc = pc;
  block->size = size;
  block->host_code = nullptr;
  block->next_block_in_page = nullptr;
  block->num_exit_links = 0;
  block->invalidated = false;
  std::memcpy(block->Instructions(), s_block_instructions.data(), sizeof(Instruction) * size);
  s_block_map[table][idx] = block;

  // add it to the tracking list for its page
  AddBlockToPageList(block);

  return block;
}

bool CPU::NewRec::RevalidateBlock(Block* block)
{
  DebugAssert(block->invalidated && !block->next_block_in_page);
  DebugAssert(BlockInRAM(block->pc));

  // blocks shouldn't be wrapping..
  const PhysicalMemoryAddress phys_addr = VirtualAddressToPhysical(block->pc);
  DebugAssert((phys_addr + (sizeof(Instruction) * block->size)) <= Bus::g_ram_size);

  // can just do a straight memcmp..
  if (std::memcmp(Bus::g_ram + phys_addr, block->Instructions(), sizeof(Instruction) * block->size) != 0)
  {
    // changed, needs recompiling
    Log_DebugPrintf("Block at PC %08X has changed and needs recompiling", block->pc);
    return false;
  }

  block->invalidated = false;
  AddBlockToPageList(block);
  return true;
}

void CPU::NewRec::CompileOrRevalidateBlock(u32 start_pc)
{
  Block* block = LookupBlock(start_pc);
  if (block)
  {
    // we should only be here if the block got invalidated
    DebugAssert(block->invalidated);
    if (RevalidateBlock(block))
    {
      SetFastMap(start_pc, block->host_code);
      BacklinkBlocks(start_pc, block->host_code);
      return;
    }

    // remove outward links from this block, since we're recompiling it
    UnlinkBlockExits(block);
  }

  block = CreateBlock(start_pc);
  if (!block)
    Panic("Failed to create block, TODO fallback to interpreter");

  block->host_code = g_compiler->CompileBlock(block);
  if (!block->host_code)
  {
    // block failed to compile
    // TODO: this shouldn't backlink
    block->host_code = &CPU::CodeCache::InterpretUncachedBlock<PGXPMode::Disabled>;
    Panic("Block failed compilation");
  }

  SetFastMap(start_pc, block->host_code);
  BacklinkBlocks(start_pc, block->host_code);
}

void CPU::NewRec::AddBlockToPageList(Block* block)
{
  if (!BlockInRAM(block->pc))
    return;

  const u32 page_idx = Bus::GetRAMCodePageIndex(VirtualMemoryAddress(block->pc));
  Bus::SetRAMCodePage(page_idx);

  auto& entry = s_page_block_lookup[page_idx];
  if (entry.second)
  {
    entry.second->next_block_in_page = block;
    entry.second = block;
  }
  else
  {
    entry.first = block;
    entry.second = block;
  }
}

u32 CPU::NewRec::CreateBlockLink(Block* block, void* code, u32 newpc)
{
  const void* dst = g_dispatcher;
  if (g_settings.cpu_recompiler_block_linking)
  {
    const Block* next_block = LookupBlock(newpc);
    dst = (next_block && !next_block->invalidated) ? next_block->host_code : g_compile_block;

    BlockLinkMap::iterator iter = s_block_links.emplace(newpc, code);
    DebugAssert(block->num_exit_links < MAX_BLOCK_EXIT_LINKS);
    block->exit_links[block->num_exit_links++] = iter;
  }

  Log_DebugPrintf("Linking %p with dst pc %08X to %p%s", code, newpc, dst,
                  (dst == g_compile_block) ? "[compiler]" : "");
  return EmitJump(code, dst);
}

void CPU::NewRec::BacklinkBlocks(u32 pc, const void* dst)
{
  if (!g_settings.cpu_recompiler_block_linking)
    return;

  const auto link_range = s_block_links.equal_range(pc);
  for (auto it = link_range.first; it != link_range.second; ++it)
  {
    Log_DebugPrintf("Backlinking %p with dst pc %08X to %p%s", it->second, pc, dst,
                    (dst == g_compile_block) ? "[compiler]" : "");
    EmitJump(it->second, dst);
  }
}

void CPU::NewRec::UnlinkBlockExits(Block* block)
{
  for (u32 i = 0; i < block->num_exit_links; i++)
    s_block_links.erase(block->exit_links[i]);
  block->num_exit_links = 0;
}

void CPU::NewRec::InvalidCodeFunction()
{
  Panic("fixme");
}

bool CPU::NewRec::Initialize()
{
  if (!s_lut_initialized)
  {
    s_lut_initialized = true;
    AllocateLUTs();
  }

  if (!g_code_buffer.Initialize(s_code_storage, sizeof(s_code_storage), 0, RECOMPILER_GUARD_SIZE))
  {
    Log_ErrorPrintf("Failed to allocate code buffer");
    return false;
  }

  CompileASMFunctions();

  ResetLUTs();

  return true;
}

void CPU::NewRec::Shutdown()
{
  g_code_buffer.Destroy();
}

void CPU::NewRec::Execute()
{
  reinterpret_cast<void (*)()>(g_enter_recompiler)();
}

void CPU::NewRec::InvalidateBlocksWithPageNumber(u32 index)
{
  DebugAssert(index < Bus::RAM_8MB_CODE_PAGE_COUNT);

  Block* block = s_page_block_lookup[index].first;
  while (block)
  {
    if (!block->invalidated)
    {
      SetFastMap(block->pc, g_compile_block);
      BacklinkBlocks(block->pc, g_compile_block);
      block->invalidated = true;
    }

    Block* next_block = block->next_block_in_page;
    block->next_block_in_page = nullptr;
    block = next_block;
  }

  s_page_block_lookup[index] = {};
  Bus::ClearRAMCodePage(index);
}

void CPU::NewRec::ClearBlocks()
{
  for (u32 i = 0; i < Bus::RAM_8MB_CODE_PAGE_COUNT; i++)
  {
    if (!s_page_block_lookup[i].first)
      continue;

    s_page_block_lookup[i] = {};
    Bus::ClearRAMCodePage(i);
  }

  s_block_links.clear();
  for (Block* block : s_blocks)
    std::free(block);
  s_blocks.clear();

  std::memset(s_lut_block_pointers.get(), 0, sizeof(Block*) * GetLUTSlotCount(false));
  ResetLUTs();
}
