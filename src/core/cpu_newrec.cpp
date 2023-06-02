// SPDX-FileCopyrightText: 2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

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
#include "system.h"
#include "util/page_fault_handler.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
Log_SetChannel(CPU::NewRec);

namespace CPU::NewRec {
using LUTRangeList = std::array<std::pair<VirtualMemoryAddress, VirtualMemoryAddress>, 9>;
using PageProtectionArray = std::array<PageProtectionInfo, Bus::RAM_8MB_CODE_PAGE_COUNT>;

static CodeLUT DecodeCodeLUTPointer(u32 slot, CodeLUT ptr);
static CodeLUT EncodeCodeLUTPointer(u32 slot, CodeLUT ptr);
static CodeLUT OffsetCodeLUTPointer(CodeLUT fake_ptr, u32 pc);

static void InvalidCodeFunction();

static void AllocateLUTs();
static void ResetLUTs();
static void InvalidateBlock(Block* block, BlockState new_state);
static void ClearBlocks();
static void CompileASMFunctions();

static u32 ReadBlockInstructions(u32 start_pc);
static void FillBlockRegInfo(Block* block);
static void SetRegAccess(InstructionInfo* inst, Reg reg, bool write);
static void AddBlockToPageList(Block* block);
static void BacklinkBlocks(u32 pc, const void* dst);
static void UnlinkBlockExits(Block* block);

static bool InitializeFastmem();
static void ShutdownFastmem();
static Common::PageFaultHandler::HandlerResult PageFaultHandler(void* exception_pc, void* fault_address, bool is_write);

// Fast map provides lookup from PC to function
// Function pointers are offset so that you don't need to subtract
CodeLUTArray g_fast_map;
static BlockLUTArray s_block_map;
static std::unique_ptr<const void*[]> s_lut_code_pointers;
static std::unique_ptr<Block*[]> s_lut_block_pointers;
static PageProtectionArray s_page_protection = {};
static std::vector<Block*> s_blocks;
static BlockLinkMap s_block_links;
static bool s_lut_initialized = false;

// for compiling
static std::vector<Instruction> s_block_instructions;

// fastmem stuff
static std::unordered_map<const void*, LoadstoreBackpatchInfo> s_fastmem_backpatch_info;
static std::unordered_set<u32> s_fastmem_faulting_pcs;

NORETURN_FUNCTION_POINTER void (*g_enter_recompiler)();
const void* g_compile_or_revalidate_block;
const void* g_discard_and_recompile_block;
const void* g_check_events_and_dispatch;
const void* g_dispatcher;
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
  for (const auto& [start, end] : GetLUTRanges())
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
    code_table_ptr[i] = reinterpret_cast<const void*>(InvalidCodeFunction);

  // Mark everything as unreachable to begin with.
  for (u32 i = 0; i < LUT_TABLE_COUNT; i++)
  {
    g_fast_map[i] = EncodeCodeLUTPointer(i, code_table_ptr);
    s_block_map[i] = nullptr;
  }
  code_table_ptr += LUT_TABLE_SIZE;

  // Allocate ranges.
  for (const auto& [start, end] : GetLUTRanges())
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
      ptr[j] = g_compile_or_revalidate_block;
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
    block =
      static_cast<Block*>(std::malloc(sizeof(Block) + (sizeof(Instruction) * size) + (sizeof(InstructionInfo) * size)));
    Assert(block);
    s_blocks.push_back(block);
  }

  block->pc = pc;
  block->size = size;
  block->host_code = nullptr;
  block->next_block_in_page = nullptr;
  block->num_exit_links = 0;
  block->state = BlockState::Valid;
  std::memcpy(block->Instructions(), s_block_instructions.data(), sizeof(Instruction) * size);
  s_block_map[table][idx] = block;

  FillBlockRegInfo(block);

  // add it to the tracking list for its page
  AddBlockToPageList(block);

  return block;
}

bool CPU::NewRec::RevalidateBlock(Block* block)
{
  DebugAssert(block->state != BlockState::Valid);
  DebugAssert(BlockInRAM(block->pc));

  if (block->state == BlockState::NeedsRecompile)
    return false;

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

  block->state = BlockState::Valid;
  AddBlockToPageList(block);
  return true;
}

void CPU::NewRec::CompileOrRevalidateBlock(u32 start_pc)
{
  // TODO: this doesn't currently handle when the cache overflows...

  Block* block = LookupBlock(start_pc);
  if (block)
  {
    // we should only be here if the block got invalidated
    DebugAssert(block->state != BlockState::Valid);
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
    block->host_code = reinterpret_cast<const void*>(&CPU::CodeCache::InterpretUncachedBlock<PGXPMode::Disabled>);
    Panic("Block failed compilation");
  }

  SetFastMap(start_pc, block->host_code);
  BacklinkBlocks(start_pc, block->host_code);
}

void CPU::NewRec::DiscardAndRecompileBlock(u32 start_pc)
{
  Log_DevPrintf("Discard block %08X with manual protection", start_pc);
  Block* block = LookupBlock(start_pc);
  DebugAssert(block && block->state == BlockState::Valid);
  InvalidateBlock(block, BlockState::NeedsRecompile);
  CompileOrRevalidateBlock(start_pc);
}

void CPU::NewRec::AddBlockToPageList(Block* block)
{
  if (!BlockInRAM(block->pc) || block->next_block_in_page)
    return;

  // TODO: what about blocks which span more than one page?
  const u32 page_idx = Bus::GetRAMCodePageIndex(block->pc);
  PageProtectionInfo& entry = s_page_protection[page_idx];
  if (entry.mode != PageProtectionMode::WriteProtected)
    return;

  Bus::SetRAMCodePage(page_idx);

  if (entry.last_block_in_page)
  {
    entry.last_block_in_page->next_block_in_page = block;
    entry.last_block_in_page = block;
  }
  else
  {
    entry.first_block_in_page = block;
    entry.last_block_in_page = block;
  }
}

void CPU::NewRec::InvalidateBlocksWithPageNumber(u32 index)
{
  DebugAssert(index < Bus::RAM_8MB_CODE_PAGE_COUNT);
  Bus::ClearRAMCodePage(index);

  BlockState new_block_state = BlockState::Invalidated;
  PageProtectionInfo& ppi = s_page_protection[index];

  const u32 frame_number = System::GetFrameNumber();
  const u32 frame_delta = frame_number - ppi.invalidate_frame;
  ppi.invalidate_count++;

  if (frame_delta >= 10)
  {
    ppi.invalidate_count = 1;
    ppi.invalidate_frame = frame_number;
  }
  else if (ppi.invalidate_count > 3)
  {
    Log_DevPrintf("%u invalidations to page %u in %u frames, switching to manual protection", ppi.invalidate_count,
                  index, frame_delta);
    ppi.mode = PageProtectionMode::ManualCheck;
    new_block_state = BlockState::NeedsRecompile;
  }

  Block* block = ppi.first_block_in_page;
  while (block)
  {
    InvalidateBlock(block, new_block_state);

    Block* next_block = block->next_block_in_page;
    block->next_block_in_page = nullptr;
    block = next_block;
  }

  ppi.first_block_in_page = nullptr;
  ppi.last_block_in_page = nullptr;
}

CPU::NewRec::PageProtectionMode CPU::NewRec::GetProtectionModeForBlock(Block* block)
{
  if (!BlockInRAM(block->pc))
    return PageProtectionMode::Unprotected;

  const u32 page_idx = Bus::GetRAMCodePageIndex(block->pc);
  const PageProtectionInfo& ppi = s_page_protection[page_idx];
  return ppi.mode;
}

u32 CPU::NewRec::CreateBlockLink(Block* block, void* code, u32 newpc)
{
  const void* dst = g_dispatcher;
  if (g_settings.cpu_recompiler_block_linking)
  {
    const Block* next_block = LookupBlock(newpc);
    dst =
      (next_block && next_block->state == BlockState::Valid) ? next_block->host_code : g_compile_or_revalidate_block;

    BlockLinkMap::iterator iter = s_block_links.emplace(newpc, code);
    DebugAssert(block->num_exit_links < MAX_BLOCK_EXIT_LINKS);
    block->exit_links[block->num_exit_links++] = iter;
  }

  Log_DebugPrintf("Linking %p with dst pc %08X to %p%s", code, newpc, dst,
                  (dst == g_compile_or_revalidate_block) ? "[compiler]" : "");
  return EmitJump(code, dst, false);
}

void CPU::NewRec::BacklinkBlocks(u32 pc, const void* dst)
{
  if (!g_settings.cpu_recompiler_block_linking)
    return;

  const auto link_range = s_block_links.equal_range(pc);
  for (auto it = link_range.first; it != link_range.second; ++it)
  {
    Log_DebugPrintf("Backlinking %p with dst pc %08X to %p%s", it->second, pc, dst,
                    (dst == g_compile_or_revalidate_block) ? "[compiler]" : "");
    EmitJump(it->second, dst, true);
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

void CPU::NewRec::CompileASMFunctions()
{
  JitCodeBuffer& buffer = CodeCache::GetCodeBuffer();
  DebugAssert(buffer.GetTotalUsed() == 0);
  const u32 asm_size = CompileASMFunctions(buffer.GetFreeCodePointer(), buffer.GetFreeCodeSpace());
  Log_ProfilePrintf("ASM functions generated %u bytes of host code", asm_size);
  buffer.CommitCode(asm_size);
}

bool CPU::NewRec::Initialize()
{
  if (!s_lut_initialized)
  {
    s_lut_initialized = true;
    AllocateLUTs();
  }

  CompileASMFunctions();
  ResetLUTs();

  if (g_settings.IsUsingFastmem() && !InitializeFastmem())
    return false;

  return true;
}

void CPU::NewRec::Shutdown()
{
  if (!s_lut_initialized)
    return;

  ClearBlocks();
  ShutdownFastmem();
}

[[noreturn]] void CPU::NewRec::Execute()
{
  g_enter_recompiler();
}

void CPU::NewRec::InvalidateBlock(Block* block, BlockState new_state)
{
  if (block->state == BlockState::Valid)
  {
    SetFastMap(block->pc, g_compile_or_revalidate_block);
    BacklinkBlocks(block->pc, g_compile_or_revalidate_block);
  }

  block->state = new_state;
}

void CPU::NewRec::InvalidateAllRAMBlocks()
{
  // TODO: maybe combine the backlink into one big instruction flush cache?

  for (Block* block : s_blocks)
  {
    if (BlockInRAM(block->pc))
      InvalidateBlock(block, BlockState::Invalidated);
  }
}

void CPU::NewRec::ClearBlocks()
{
  for (u32 i = 0; i < Bus::RAM_8MB_CODE_PAGE_COUNT; i++)
  {
    PageProtectionInfo& ppi = s_page_protection[i];
    if (ppi.mode == PageProtectionMode::WriteProtected && ppi.first_block_in_page)
      Bus::ClearRAMCodePage(i);

    ppi = {};
  }

  s_fastmem_backpatch_info.clear();
  s_fastmem_faulting_pcs.clear();
  s_block_links.clear();
  for (Block* block : s_blocks)
    std::free(block);
  s_blocks.clear();

  std::memset(s_lut_block_pointers.get(), 0, sizeof(Block*) * GetLUTSlotCount(false));
}

void CPU::NewRec::Reset()
{
  ClearBlocks();
  CompileASMFunctions();
  ResetLUTs();

  if (g_settings.IsUsingFastmem())
    CPU::UpdateFastmemBase();
}

bool CPU::NewRec::InitializeFastmem()
{
  const CPUFastmemMode mode = g_settings.cpu_fastmem_mode;
  Assert(mode == CPUFastmemMode::MMap);

  JitCodeBuffer& buffer = CodeCache::GetCodeBuffer();
  if (!Common::PageFaultHandler::InstallHandler(&g_fast_map, buffer.GetCodePointer(), buffer.GetTotalSize(),
                                                &PageFaultHandler))
  {
    Log_ErrorPrintf("Failed to install page fault handler");
    return false;
  }

  Bus::UpdateFastmemViews(mode);
  CPU::UpdateFastmemBase();
  return true;
}

void CPU::NewRec::ShutdownFastmem()
{
  Common::PageFaultHandler::RemoveHandler(&g_fast_map);
  Bus::UpdateFastmemViews(CPUFastmemMode::Disabled);
  CPU::UpdateFastmemBase();
}

void CPU::NewRec::AddLoadStoreInfo(void* code_address, u32 code_size, u32 guest_pc, TickCount cycles, u32 gpr_bitmask,
                                   u8 address_register, u8 data_register, MemoryAccessSize size, bool is_signed,
                                   bool is_load)
{
  DebugAssert(code_size < std::numeric_limits<u8>::max());
  DebugAssert(cycles >= 0 && cycles < std::numeric_limits<u16>::max());

  auto iter = s_fastmem_backpatch_info.find(code_address);
  if (iter != s_fastmem_backpatch_info.end())
    s_fastmem_backpatch_info.erase(iter);

  const LoadstoreBackpatchInfo info{
    guest_pc,  gpr_bitmask, static_cast<u16>(cycles),   address_register,  data_register, static_cast<u16>(size),
    is_signed, is_load,     static_cast<u8>(code_size), static_cast<u8>(0)};
  s_fastmem_backpatch_info.emplace(code_address, info);
}

Common::PageFaultHandler::HandlerResult CPU::NewRec::PageFaultHandler(void* exception_pc, void* fault_address,
                                                                      bool is_write)
{
  if (static_cast<u8*>(fault_address) < g_state.fastmem_base ||
      (static_cast<u8*>(fault_address) - g_state.fastmem_base) >= static_cast<ptrdiff_t>(Bus::FASTMEM_REGION_SIZE))
  {
    return Common::PageFaultHandler::HandlerResult::ExecuteNextHandler;
  }

  const PhysicalMemoryAddress guest_address =
    static_cast<PhysicalMemoryAddress>(static_cast<ptrdiff_t>(static_cast<u8*>(fault_address) - g_state.fastmem_base));

  Log_DevPrintf("Page fault handler invoked at PC=%p Address=%p %s, fastmem offset 0x%08X", exception_pc, fault_address,
                is_write ? "(write)" : "(read)", guest_address);

  auto iter = s_fastmem_backpatch_info.find(exception_pc);
  if (iter == s_fastmem_backpatch_info.end())
  {
    Log_ErrorPrintf("No backpatch info found for %p", exception_pc);
    return Common::PageFaultHandler::HandlerResult::ExecuteNextHandler;
  }

  // if we're writing to ram, let it go through a few times, and use manual block protection to sort it out
  LoadstoreBackpatchInfo& info = iter->second;
  if (is_write && !g_state.cop0_regs.sr.Isc && Bus::IsRAMAddress(guest_address) && info.fault_count < 10)
  {
    Log_DevPrintf("Ignoring fault due to RAM write");
    InvalidateBlocksWithPageNumber(Bus::GetRAMCodePageIndex(guest_address));
    info.fault_count++;
    return Common::PageFaultHandler::HandlerResult::ContinueExecution;
  }

  Log_DevPrintf("Backpatching %s at %p[%u] (pc %08X addr %08X): Bitmask %08X Addr %u Data %u Size %u Signed %02X",
                info.is_load ? "load" : "store", exception_pc, info.code_size, info.guest_pc, guest_address,
                info.gpr_bitmask, info.address_register, info.data_register, info.AccessSizeInBytes(), info.is_signed);

  // remove the cycles we added for the memory read, then take them off again after the backpatch
  // the normal rec path will add the ram read ticks later, so we need to take them off at the end
  DebugAssert(!info.is_load || info.cycles >= Bus::RAM_READ_TICKS);
  const TickCount cycles_to_add =
    static_cast<TickCount>(static_cast<u32>(info.cycles)) - (info.is_load ? Bus::RAM_READ_TICKS : 0);
  const TickCount cycles_to_remove = static_cast<TickCount>(static_cast<u32>(info.cycles));

  JitCodeBuffer& buffer = CodeCache::GetCodeBuffer();
  const u32 thunk_size =
    BackpatchLoadStore(buffer.GetFreeFarCodePointer(), buffer.GetFreeFarCodeSpace(), exception_pc, info.code_size,
                       cycles_to_add, cycles_to_remove, info.gpr_bitmask, info.address_register, info.data_register,
                       info.AccessSize(), info.is_signed, info.is_load);
  buffer.CommitFarCode(thunk_size);

  // TODO: queue block for recompilation later

  // and store the pc in the faulting list, so that we don't emit another fastmem loadstore
  s_fastmem_faulting_pcs.insert(info.guest_pc);
  s_fastmem_backpatch_info.erase(iter);
  return Common::PageFaultHandler::HandlerResult::ContinueExecution;
}

// TODO: move this into the compiler

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

void CPU::NewRec::SetRegAccess(InstructionInfo* inst, Reg reg, bool write)
{
  if (reg == Reg::zero)
    return;

  if (!write)
  {
    for (u32 i = 0; i < std::size(inst->read_reg); i++)
    {
      if (inst->read_reg[i] == Reg::zero)
      {
        inst->read_reg[i] = reg;
        break;
      }
    }
  }
  else
  {
#if 0
    for (u32 i = 0; i < std::size(inst->write_reg); i++)
    {
      if (inst->write_reg[i] == Reg::zero)
      {
        inst->write_reg[i] = reg;
        break;
      }
    }
#endif
  }
}

#define BackpropSetReads(reg)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(inst->reg_flags[static_cast<u8>(reg)] & RI_USED))                                                            \
      inst->reg_flags[static_cast<u8>(reg)] |= RI_LASTUSE;                                                             \
    prev->reg_flags[static_cast<u8>(reg)] |= RI_LIVE | RI_USED;                                                        \
    inst->reg_flags[static_cast<u8>(reg)] |= RI_USED;                                                                  \
    SetRegAccess(inst, reg, false);                                                                                    \
  } while (0)

#define BackpropSetWrites(reg)                                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    prev->reg_flags[static_cast<u8>(reg)] &= ~(RI_LIVE | RI_USED);                                                     \
    if (!(inst->reg_flags[static_cast<u8>(reg)] & RI_USED))                                                            \
      inst->reg_flags[static_cast<u8>(reg)] |= RI_LASTUSE;                                                             \
    inst->reg_flags[static_cast<u8>(reg)] |= RI_USED;                                                                  \
    SetRegAccess(inst, reg, true);                                                                                     \
  } while (0)

// TODO: memory loads should be delayed one instruction because of stupid load delays.
#define BackpropSetWritesDelayed(reg) BackpropSetWrites(reg)

void CPU::NewRec::FillBlockRegInfo(Block* block)
{
  const Instruction* iinst = block->Instructions() + (block->size - 1);
  InstructionInfo* const start = block->InstructionsInfo();
  InstructionInfo* inst = start + (block->size - 1);
  std::memset(inst->reg_flags, RI_LIVE, sizeof(inst->reg_flags));
  std::memset(inst->read_reg, 0, sizeof(inst->read_reg));
  // std::memset(inst->write_reg, 0, sizeof(inst->write_reg));

  while (inst != start)
  {
    InstructionInfo* prev = inst - 1;
    std::memcpy(prev, inst, sizeof(InstructionInfo));

    const Reg rs = iinst->r.rs;
    const Reg rt = iinst->r.rt;

    switch (iinst->op)
    {
      case InstructionOp::funct:
      {
        const Reg rd = iinst->r.rd;

        switch (iinst->r.funct)
        {
          case InstructionFunct::sll:
          case InstructionFunct::srl:
          case InstructionFunct::sra:
            BackpropSetWrites(rd);
            BackpropSetReads(rt);
            break;

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
            BackpropSetWrites(rd);
            BackpropSetReads(rt);
            BackpropSetReads(rs);
            break;

          case InstructionFunct::jr:
            BackpropSetReads(rs);
            break;

          case InstructionFunct::jalr:
            BackpropSetReads(rs);
            BackpropSetWrites(rd);
            break;

          case InstructionFunct::mfhi:
            BackpropSetWrites(rd);
            BackpropSetReads(Reg::hi);
            break;

          case InstructionFunct::mflo:
            BackpropSetWrites(rd);
            BackpropSetReads(Reg::lo);
            break;

          case InstructionFunct::mthi:
            BackpropSetWrites(Reg::hi);
            BackpropSetReads(rs);
            break;

          case InstructionFunct::mtlo:
            BackpropSetWrites(Reg::lo);
            BackpropSetReads(rs);
            break;

          case InstructionFunct::mult:
          case InstructionFunct::multu:
          case InstructionFunct::div:
          case InstructionFunct::divu:
            BackpropSetWrites(Reg::hi);
            BackpropSetWrites(Reg::lo);
            BackpropSetReads(rs);
            BackpropSetReads(rt);
            break;

          case InstructionFunct::syscall:
          case InstructionFunct::break_:
            break;

          default:
            Log_ErrorPrintf("Unknown funct %u", static_cast<u32>(iinst->r.funct.GetValue()));
            break;
        }
      }
      break;

      case InstructionOp::b:
      {
        if ((static_cast<u8>(iinst->i.rt.GetValue()) & u8(0x1E)) == u8(0x10))
          BackpropSetWrites(Reg::ra);
        BackpropSetReads(rs);
      }
      break;

      case InstructionOp::j:
        break;

      case InstructionOp::jal:
        BackpropSetWrites(Reg::ra);
        break;

      case InstructionOp::beq:
      case InstructionOp::bne:
        BackpropSetReads(rs);
        BackpropSetReads(rt);
        break;

      case InstructionOp::blez:
      case InstructionOp::bgtz:
        BackpropSetReads(rs);
        break;

      case InstructionOp::addi:
      case InstructionOp::addiu:
      case InstructionOp::slti:
      case InstructionOp::sltiu:
      case InstructionOp::andi:
      case InstructionOp::ori:
      case InstructionOp::xori:
        BackpropSetWrites(rt);
        BackpropSetReads(rs);
        break;

      case InstructionOp::lui:
        BackpropSetWrites(rt);
        break;

      case InstructionOp::lb:
      case InstructionOp::lh:
      case InstructionOp::lw:
      case InstructionOp::lbu:
      case InstructionOp::lhu:
        BackpropSetWritesDelayed(rt);
        BackpropSetReads(rs);
        break;

      case InstructionOp::lwl:
      case InstructionOp::lwr:
        BackpropSetWritesDelayed(rt);
        BackpropSetReads(rs);
        BackpropSetReads(rt);
        break;

      case InstructionOp::sb:
      case InstructionOp::sh:
      case InstructionOp::swl:
      case InstructionOp::sw:
      case InstructionOp::swr:
        BackpropSetReads(rt);
        BackpropSetReads(rs);
        break;

      case InstructionOp::cop0:
      case InstructionOp::cop2:
      {
        if (iinst->cop.IsCommonInstruction())
        {
          switch (iinst->cop.CommonOp())
          {
            case CopCommonInstruction::mfcn:
            case CopCommonInstruction::cfcn:
              BackpropSetWritesDelayed(rt);
              break;

            case CopCommonInstruction::mtcn:
            case CopCommonInstruction::ctcn:
              BackpropSetReads(rt);
              break;
          }
        }
        break;

        case InstructionOp::lwc2:
        case InstructionOp::swc2:
          BackpropSetReads(rs);
          BackpropSetReads(rt);
          break;

        default:
          Log_ErrorPrintf("Unknown op %u", static_cast<u32>(iinst->r.funct.GetValue()));
          break;
      }
    } // end switch

    inst--;
    iinst--;
  } // end while
}
