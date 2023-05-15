// SPDX-FileCopyrightText: 2019-2023 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "system.h"
#include "IconsFontAwesome5.h"
#include "achievements.h"
#include "bios.h"
#include "bus.h"
#include "cdrom.h"
#include "cheats.h"
#include "common/error.h"
#include "common/file_system.h"
#include "common/log.h"
#include "common/make_array.h"
#include "common/path.h"
#include "common/string_util.h"
#include "common/threading.h"
#include "controller.h"
#include "cpu_code_cache.h"
#include "cpu_core.h"
#include "dma.h"
#include "fmt/chrono.h"
#include "fmt/format.h"
#include "game_database.h"
#include "gpu.h"
#include "gte.h"
#include "host.h"
#include "host_display.h"
#include "host_interface_progress_callback.h"
#include "host_settings.h"
#include "interrupt_controller.h"
#include "libcrypt_serials.h"
#include "mdec.h"
#include "memory_card.h"
#include "multitap.h"
#include "pad.h"
#include "pcdrv.h"
#include "pgxp.h"
#include "psf_loader.h"
#include "save_state_version.h"
#include "sio.h"
#include "spu.h"
#include "texture_replacements.h"
#include "timers.h"
#include "util/audio_stream.h"
#include "util/cd_image.h"
#include "util/ini_settings_interface.h"
#include "util/iso_reader.h"
#include "util/state_wrapper.h"
#include "xxhash.h"
#include <cctype>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <limits>
#include <thread>
Log_SetChannel(System);

#ifdef _WIN32
#include "common/windows_headers.h"
#include <mmsystem.h>
#endif

// #define PROFILE_MEMORY_SAVE_STATES 1

SystemBootParameters::SystemBootParameters() = default;

SystemBootParameters::SystemBootParameters(const SystemBootParameters&) = default;

SystemBootParameters::SystemBootParameters(SystemBootParameters&& other) = default;

SystemBootParameters::SystemBootParameters(std::string filename_) : filename(std::move(filename_)) {}

SystemBootParameters::~SystemBootParameters() = default;

struct MemorySaveState
{
  std::unique_ptr<GPUTexture> vram_texture;
  std::unique_ptr<GrowableMemoryByteStream> state_stream;
};

namespace System {
static std::optional<ExtendedSaveStateInfo> InternalGetExtendedSaveStateInfo(ByteStream* stream);
static bool InternalSaveState(ByteStream* state, u32 screenshot_size = 256,
                              u32 compression_method = SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE);
static bool SaveMemoryState(MemorySaveState* mss);
static bool LoadMemoryState(const MemorySaveState& mss);

static bool LoadEXE(const char* filename);

static std::string GetExecutableNameForImage(ISOReader& iso, bool strip_subdirectories);
static bool ReadExecutableFromImage(ISOReader& iso, std::string* out_executable_name,
                                    std::vector<u8>* out_executable_data);

static void StallCPU(TickCount ticks);

static bool LoadBIOS();
static void InternalReset();
static void ClearRunningGame();
static void DestroySystem();
static std::string GetMediaPathFromSaveState(const char* path);
static bool DoLoadState(ByteStream* stream, bool force_software_renderer, bool update_display);
static bool DoState(StateWrapper& sw, GPUTexture** host_texture, bool update_display, bool is_memory_state);
static void DoRunFrame();
static bool CreateGPU(GPURenderer renderer);
static bool SaveUndoLoadState();

static void SetRewinding(bool enabled);
static bool SaveRewindState();
static void DoRewind();

static void SaveRunaheadState();
static void DoRunahead();

static void DoMemorySaveStates();

static bool Initialize(bool force_software_renderer);
static bool FastForwardToFirstFrame();

static bool UpdateGameSettingsLayer();
static void UpdateRunningGame(const char* path, CDImage* image, bool booting);
static bool CheckForSBIFile(CDImage* image);
static std::unique_ptr<MemoryCard> GetMemoryCardForSlot(u32 slot, MemoryCardType type);

static void SetTimerResolutionIncreased(bool enabled);
} // namespace System

static constexpr const float PERFORMANCE_COUNTER_UPDATE_INTERVAL = 1.0f;
static constexpr const char FALLBACK_EXE_NAME[] = "PSX.EXE";

static std::unique_ptr<INISettingsInterface> s_game_settings_interface;
static std::unique_ptr<INISettingsInterface> s_input_settings_interface;
static std::string s_input_profile_name;

static System::State s_state = System::State::Shutdown;
static std::atomic_bool s_startup_cancelled{false};

static ConsoleRegion s_region = ConsoleRegion::NTSC_U;
TickCount System::g_ticks_per_second = System::MASTER_CLOCK;
static TickCount s_max_slice_ticks = System::MASTER_CLOCK / 10;
static u32 s_frame_number = 1;
static u32 s_internal_frame_number = 1;
static const BIOS::ImageInfo* s_bios_image_info = nullptr;
static BIOS::Hash s_bios_hash = {};

static std::string s_running_game_path;
static std::string s_running_game_serial;
static std::string s_running_game_title;
static System::GameHash s_running_game_hash;
static bool s_running_unknown_game;
static bool s_was_fast_booted;

static float s_throttle_frequency = 60.0f;
static float s_target_speed = 1.0f;
static Common::Timer::Value s_frame_period = 0;
static Common::Timer::Value s_next_frame_time = 0;

static bool s_frame_step_request = false;
static bool s_fast_forward_enabled = false;
static bool s_turbo_enabled = false;
static bool s_throttler_enabled = true;
static bool s_display_all_frames = true;
static bool s_syncing_to_host = false;

static float s_average_frame_time_accumulator = 0.0f;
static float s_minimum_frame_time_accumulator = 0.0f;
static float s_maximum_frame_time_accumulator = 0.0f;

static float s_vps = 0.0f;
static float s_fps = 0.0f;
static float s_speed = 0.0f;
static float s_minimum_frame_time = 0.0f;
static float s_maximum_frame_time = 0.0f;
static float s_average_frame_time = 0.0f;
static float s_cpu_thread_usage = 0.0f;
static float s_cpu_thread_time = 0.0f;
static float s_sw_thread_usage = 0.0f;
static float s_sw_thread_time = 0.0f;
static float s_average_gpu_time = 0.0f;
static float s_accumulated_gpu_time = 0.0f;
static float s_gpu_usage = 0.0f;
static System::FrameTimeHistory s_frame_time_history;
static u32 s_frame_time_history_pos = 0;
static u32 s_last_frame_number = 0;
static u32 s_last_internal_frame_number = 0;
static u32 s_last_global_tick_counter = 0;
static u64 s_last_cpu_time = 0;
static u64 s_last_sw_time = 0;
static u32 s_presents_since_last_update = 0;
static Common::Timer s_fps_timer;
static Common::Timer s_frame_timer;
static Threading::ThreadHandle s_cpu_thread_handle;

static std::unique_ptr<CheatList> s_cheat_list;

// temporary save state, created when loading, used to undo load state
static std::unique_ptr<ByteStream> m_undo_load_state;

static bool s_memory_saves_enabled = false;

static std::deque<MemorySaveState> s_rewind_states;
static s32 s_rewind_load_frequency = -1;
static s32 s_rewind_load_counter = -1;
static s32 s_rewind_save_frequency = -1;
static s32 s_rewind_save_counter = -1;
static bool s_rewinding_first_save = false;

static std::deque<MemorySaveState> s_runahead_states;
static bool s_runahead_replay_pending = false;
static u32 s_runahead_frames = 0;

static TinyString GetTimestampStringForFileName()
{
  return TinyString::FromFmt("{:%Y-%m-%d_%H-%M-%S}", fmt::localtime(std::time(nullptr)));
}

System::State System::GetState()
{
  return s_state;
}

void System::SetState(State new_state)
{
  if (s_state == new_state)
    return;

  Assert(s_state == State::Paused || s_state == State::Running);
  Assert(new_state == State::Paused || new_state == State::Running);
  s_state = new_state;

  if (new_state == State::Paused)
    CPU::ForceDispatcherExit();
}

bool System::IsRunning()
{
  return s_state == State::Running;
}

bool System::IsPaused()
{
  return s_state == State::Paused;
}

bool System::IsShutdown()
{
  return s_state == State::Shutdown;
}

bool System::IsValid()
{
  return s_state == State::Running || s_state == State::Paused;
}

bool System::IsStartupCancelled()
{
  return s_startup_cancelled.load();
}

void System::CancelPendingStartup()
{
  if (s_state == State::Starting)
    s_startup_cancelled.store(true);
}

ConsoleRegion System::GetRegion()
{
  return s_region;
}

DiscRegion System::GetDiscRegion()
{
  return CDROM::GetDiscRegion();
}

bool System::IsPALRegion()
{
  return s_region == ConsoleRegion::PAL;
}

TickCount System::GetMaxSliceTicks()
{
  return s_max_slice_ticks;
}

void System::UpdateOverclock()
{
  g_ticks_per_second = ScaleTicksToOverclock(MASTER_CLOCK);
  s_max_slice_ticks = ScaleTicksToOverclock(MASTER_CLOCK / 10);
  SPU::CPUClockChanged();
  CDROM::CPUClockChanged();
  g_gpu->CPUClockChanged();
  Timers::CPUClocksChanged();
  UpdateThrottlePeriod();
}

u32 System::GetFrameNumber()
{
  return s_frame_number;
}

u32 System::GetInternalFrameNumber()
{
  return s_internal_frame_number;
}

void System::FrameDone()
{
  s_frame_number++;
  CPU::g_state.frame_done = true;
  CPU::g_state.downcount = 0;
}

void System::IncrementInternalFrameNumber()
{
  s_internal_frame_number++;
}

const std::string& System::GetDiscPath()
{
  return s_running_game_path;
}
const std::string& System::GetGameSerial()
{
  return s_running_game_serial;
}

const std::string& System::GetGameTitle()
{
  return s_running_game_title;
}

System::GameHash System::GetGameHash()
{
  return s_running_game_hash;
}

bool System::IsRunningUnknownGame()
{
  return s_running_unknown_game;
}

bool System::WasFastBooted()
{
  return s_was_fast_booted;
}

const BIOS::ImageInfo* System::GetBIOSImageInfo()
{
  return s_bios_image_info;
}

const BIOS::Hash& System::GetBIOSHash()
{
  return s_bios_hash;
}

float System::GetFPS()
{
  return s_fps;
}
float System::GetVPS()
{
  return s_vps;
}
float System::GetEmulationSpeed()
{
  return s_speed;
}
float System::GetAverageFrameTime()
{
  return s_average_frame_time;
}
float System::GetMinimumFrameTime()
{
  return s_minimum_frame_time;
}
float System::GetMaximumFrameTime()
{
  return s_maximum_frame_time;
}
float System::GetThrottleFrequency()
{
  return s_throttle_frequency;
}
float System::GetCPUThreadUsage()
{
  return s_cpu_thread_usage;
}
float System::GetCPUThreadAverageTime()
{
  return s_cpu_thread_time;
}
float System::GetSWThreadUsage()
{
  return s_sw_thread_usage;
}
float System::GetSWThreadAverageTime()
{
  return s_sw_thread_time;
}
float System::GetGPUUsage()
{
  return s_gpu_usage;
}
float System::GetGPUAverageTime()
{
  return s_average_gpu_time;
}
const System::FrameTimeHistory& System::GetFrameTimeHistory()
{
  return s_frame_time_history;
}
u32 System::GetFrameTimeHistoryPos()
{
  return s_frame_time_history_pos;
}

bool System::IsExeFileName(const std::string_view& path)
{
  return (StringUtil::EndsWithNoCase(path, ".exe") || StringUtil::EndsWithNoCase(path, ".psexe") ||
          StringUtil::EndsWithNoCase(path, ".ps-exe"));
}

bool System::IsPsfFileName(const std::string_view& path)
{
  return (StringUtil::EndsWithNoCase(path, ".psf") || StringUtil::EndsWithNoCase(path, ".minipsf"));
}

bool System::IsLoadableFilename(const std::string_view& path)
{
  static constexpr auto extensions = make_array(".bin", ".cue", ".img", ".iso", ".chd", ".ecm", ".mds", // discs
                                                ".exe", ".psexe", ".ps-exe",                            // exes
                                                ".psf", ".minipsf",                                     // psf
                                                ".m3u",                                                 // playlists
                                                ".pbp");

  for (const char* test_extension : extensions)
  {
    if (StringUtil::EndsWithNoCase(path, test_extension))
      return true;
  }

  return false;
}

bool System::IsSaveStateFilename(const std::string_view& path)
{
  return StringUtil::EndsWithNoCase(path, ".sav");
}

ConsoleRegion System::GetConsoleRegionForDiscRegion(DiscRegion region)
{
  switch (region)
  {
    case DiscRegion::NTSC_J:
      return ConsoleRegion::NTSC_J;

    case DiscRegion::NTSC_U:
    case DiscRegion::Other:
    case DiscRegion::NonPS1:
    default:
      return ConsoleRegion::NTSC_U;

    case DiscRegion::PAL:
      return ConsoleRegion::PAL;
  }
}

std::string System::GetGameHashId(GameHash hash)
{
  return StringUtil::StdStringFromFormat("HASH-%" PRIX64, hash);
}

bool System::GetGameDetailsFromImage(CDImage* cdi, std::string* out_id, GameHash* out_hash)
{
  ISOReader iso;
  if (!iso.Open(cdi, 1))
  {
    if (out_id)
      out_id->clear();
    if (out_hash)
      *out_hash = 0;
    return false;
  }

  std::string id;
  std::string exe_name;
  std::vector<u8> exe_buffer;
  if (!ReadExecutableFromImage(iso, &exe_name, &exe_buffer))
  {
    if (out_id)
      out_id->clear();
    if (out_hash)
      *out_hash = 0;
    return false;
  }

  // Always compute the hash.
  const u32 track_1_length = cdi->GetTrackLength(1);
  XXH64_state_t* state = XXH64_createState();
  XXH64_reset(state, 0x4242D00C);
  XXH64_update(state, exe_name.c_str(), exe_name.size());
  XXH64_update(state, exe_buffer.data(), exe_buffer.size());
  XXH64_update(state, &iso.GetPVD(), sizeof(ISOReader::ISOPrimaryVolumeDescriptor));
  XXH64_update(state, &track_1_length, sizeof(track_1_length));
  const GameHash hash = XXH64_digest(state);
  XXH64_freeState(state);
  Log_DevPrintf("Hash for '%s' - %" PRIX64, exe_name.c_str(), hash);

  if (exe_name != FALLBACK_EXE_NAME)
  {
    // Strip off any subdirectories.
    const std::string::size_type slash = exe_name.rfind('\\');
    if (slash != std::string::npos)
      id = std::string_view(exe_name).substr(slash + 1);
    else
      id = exe_name;

    // SCES_123.45 -> SCES-12345
    for (std::string::size_type pos = 0; pos < id.size();)
    {
      if (id[pos] == '.')
      {
        id.erase(pos, 1);
        continue;
      }

      if (id[pos] == '_')
        id[pos] = '-';
      else
        id[pos] = static_cast<char>(std::toupper(id[pos]));

      pos++;
    }
  }

  if (out_id)
  {
    if (id.empty())
      *out_id = GetGameHashId(hash);
    else
      *out_id = std::move(id);
  }

  if (out_hash)
    *out_hash = hash;

  return true;
}

std::string System::GetExecutableNameForImage(ISOReader& iso, bool strip_subdirectories)
{
  // Read SYSTEM.CNF
  std::vector<u8> system_cnf_data;
  if (!iso.ReadFile("SYSTEM.CNF", &system_cnf_data))
    return FALLBACK_EXE_NAME;

  // Parse lines
  std::vector<std::pair<std::string, std::string>> lines;
  std::pair<std::string, std::string> current_line;
  bool reading_value = false;
  for (size_t pos = 0; pos < system_cnf_data.size(); pos++)
  {
    const char ch = static_cast<char>(system_cnf_data[pos]);
    if (ch == '\r' || ch == '\n')
    {
      if (!current_line.first.empty())
      {
        lines.push_back(std::move(current_line));
        current_line = {};
        reading_value = false;
      }
    }
    else if (ch == ' ' || (ch >= 0x09 && ch <= 0x0D))
    {
      continue;
    }
    else if (ch == '=' && !reading_value)
    {
      reading_value = true;
    }
    else
    {
      if (reading_value)
        current_line.second.push_back(ch);
      else
        current_line.first.push_back(ch);
    }
  }

  if (!current_line.first.empty())
    lines.push_back(std::move(current_line));

  // Find the BOOT line
  auto iter = std::find_if(lines.begin(), lines.end(),
                           [](const auto& it) { return StringUtil::Strcasecmp(it.first.c_str(), "boot") == 0; });
  if (iter == lines.end())
  {
    // Fallback to PSX.EXE
    return FALLBACK_EXE_NAME;
  }

  std::string code = iter->second;
  std::string::size_type pos;
  if (strip_subdirectories)
  {
    // cdrom:\SCES_123.45;1
    pos = code.rfind('\\');
    if (pos != std::string::npos)
    {
      code.erase(0, pos + 1);
    }
    else
    {
      // cdrom:SCES_123.45;1
      pos = code.rfind(':');
      if (pos != std::string::npos)
        code.erase(0, pos + 1);
    }
  }
  else
  {
    if (code.compare(0, 6, "cdrom:") == 0)
      code.erase(0, 6);
    else
      Log_WarningPrintf("Unknown prefix in executable path: '%s'", code.c_str());

    // remove leading slashes
    while (code[0] == '/' || code[0] == '\\')
      code.erase(0, 1);
  }

  // strip off ; or version number
  pos = code.rfind(';');
  if (pos != std::string::npos)
    code.erase(pos);

  return code;
}

std::string System::GetExecutableNameForImage(CDImage* cdi, bool strip_subdirectories)
{
  ISOReader iso;
  if (!iso.Open(cdi, 1))
    return {};

  return GetExecutableNameForImage(iso, strip_subdirectories);
}

bool System::ReadExecutableFromImage(CDImage* cdi, std::string* out_executable_name,
                                     std::vector<u8>* out_executable_data)
{
  ISOReader iso;
  if (!iso.Open(cdi, 1))
    return false;

  return ReadExecutableFromImage(iso, out_executable_name, out_executable_data);
}

bool System::ReadExecutableFromImage(ISOReader& iso, std::string* out_executable_name,
                                     std::vector<u8>* out_executable_data)
{
  const std::string executable_path = GetExecutableNameForImage(iso, false);
  Log_DevPrintf("Executable path: '%s'", executable_path.c_str());
  if (!executable_path.empty() && out_executable_data)
  {
    if (!iso.ReadFile(executable_path.c_str(), out_executable_data))
    {
      Log_ErrorPrintf("Failed to read executable '%s' from disc", executable_path.c_str());
      return false;
    }
  }

  if (out_executable_name)
    *out_executable_name = std::move(executable_path);

  return true;
}

DiscRegion System::GetRegionForSerial(std::string_view serial)
{
  std::string prefix;
  for (size_t pos = 0; pos < serial.length(); pos++)
  {
    const int ch = std::tolower(serial[pos]);
    if (ch < 'a' || ch > 'z')
      break;

    prefix.push_back(static_cast<char>(ch));
  }

  if (prefix == "sces" || prefix == "sced" || prefix == "sles" || prefix == "sled")
    return DiscRegion::PAL;
  else if (prefix == "scps" || prefix == "slps" || prefix == "slpm" || prefix == "sczs" || prefix == "papx")
    return DiscRegion::NTSC_J;
  else if (prefix == "scus" || prefix == "slus")
    return DiscRegion::NTSC_U;
  else
    return DiscRegion::Other;
}

DiscRegion System::GetRegionFromSystemArea(CDImage* cdi)
{
  // The license code is on sector 4 of the disc.
  u8 sector[CDImage::DATA_SECTOR_SIZE];
  if (!cdi->Seek(1, 4) || cdi->Read(CDImage::ReadMode::DataOnly, 1, sector) != 1)
    return DiscRegion::Other;

  static constexpr char ntsc_u_string[] = "          Licensed  by          Sony Computer Entertainment Amer  ica ";
  static constexpr char ntsc_j_string[] = "          Licensed  by          Sony Computer Entertainment Inc.";
  static constexpr char pal_string[] = "          Licensed  by          Sony Computer Entertainment Euro pe";

  // subtract one for the terminating null
  if (std::equal(ntsc_u_string, ntsc_u_string + countof(ntsc_u_string) - 1, sector))
    return DiscRegion::NTSC_U;
  else if (std::equal(ntsc_j_string, ntsc_j_string + countof(ntsc_j_string) - 1, sector))
    return DiscRegion::NTSC_J;
  else if (std::equal(pal_string, pal_string + countof(pal_string) - 1, sector))
    return DiscRegion::PAL;
  else
    return DiscRegion::Other;
}

DiscRegion System::GetRegionForImage(CDImage* cdi)
{
  const DiscRegion system_area_region = GetRegionFromSystemArea(cdi);
  if (system_area_region != DiscRegion::Other)
    return system_area_region;

  ISOReader iso;
  if (!iso.Open(cdi, 1))
    return DiscRegion::NonPS1;

  // The executable must exist, because this just returns PSX.EXE if it doesn't.
  const std::string exename = GetExecutableNameForImage(iso, false);
  if (exename.empty() || !iso.FileExists(exename.c_str()))
    return DiscRegion::NonPS1;

  // Strip off any subdirectories.
  const std::string::size_type slash = exename.rfind('\\');
  if (slash != std::string::npos)
    return GetRegionForSerial(std::string_view(exename).substr(slash + 1));
  else
    return GetRegionForSerial(exename);
}

DiscRegion System::GetRegionForExe(const char* path)
{
  auto fp = FileSystem::OpenManagedCFile(path, "rb");
  if (!fp)
    return DiscRegion::Other;

  BIOS::PSEXEHeader header;
  if (std::fread(&header, sizeof(header), 1, fp.get()) != 1)
    return DiscRegion::Other;

  return BIOS::GetPSExeDiscRegion(header);
}

DiscRegion System::GetRegionForPsf(const char* path)
{
  PSFLoader::File psf;
  if (!psf.Load(path))
    return DiscRegion::Other;

  return psf.GetRegion();
}

std::optional<DiscRegion> System::GetRegionForPath(const char* image_path)
{
  if (IsExeFileName(image_path))
    return GetRegionForExe(image_path);
  else if (IsPsfFileName(image_path))
    return GetRegionForPsf(image_path);

  std::unique_ptr<CDImage> cdi = CDImage::Open(image_path, false, nullptr);
  if (!cdi)
    return {};

  return GetRegionForImage(cdi.get());
}

std::string System::GetGameSettingsPath(const std::string_view& game_serial)
{
  const std::string sanitized_serial(Path::SanitizeFileName(game_serial));
  return Path::Combine(EmuFolders::GameSettings, fmt::format("{}.ini", sanitized_serial));
}

std::string System::GetInputProfilePath(const std::string_view& name)
{
  return Path::Combine(EmuFolders::InputProfiles, fmt::format("{}.ini", name));
}

bool System::RecreateGPU(GPURenderer renderer, bool force_recreate_display, bool update_display /* = true*/)
{
  ClearMemorySaveStates();
  g_gpu->RestoreGraphicsAPIState();

  // save current state
  std::unique_ptr<ByteStream> state_stream = ByteStream::CreateGrowableMemoryStream();
  StateWrapper sw(state_stream.get(), StateWrapper::Mode::Write, SAVE_STATE_VERSION);
  const bool state_valid = g_gpu->DoState(sw, nullptr, false) && TimingEvents::DoState(sw);
  if (!state_valid)
    Log_ErrorPrintf("Failed to save old GPU state when switching renderers");

  g_gpu->ResetGraphicsAPIState();

  // create new renderer
  g_gpu.reset();
  if (force_recreate_display)
    Host::ReleaseHostDisplay();

  if (!CreateGPU(renderer))
  {
    if (!IsStartupCancelled())
      Host::ReportErrorAsync("Error", "Failed to recreate GPU.");

    DestroySystem();
    return false;
  }

  if (state_valid)
  {
    state_stream->SeekAbsolute(0);
    sw.SetMode(StateWrapper::Mode::Read);
    g_gpu->RestoreGraphicsAPIState();
    g_gpu->DoState(sw, nullptr, update_display);
    TimingEvents::DoState(sw);
    g_gpu->ResetGraphicsAPIState();
  }

  // fix up vsync etc
  UpdateSpeedLimiterState();
  return true;
}

void System::LoadSettings(bool display_osd_messages)
{
  std::unique_lock<std::mutex> lock = Host::GetSettingsLock();
  SettingsInterface& si = *Host::GetSettingsInterface();
  g_settings.Load(si);
  Host::LoadSettings(si, lock);

  // apply compatibility settings
  if (g_settings.apply_compatibility_settings && !s_running_game_serial.empty())
  {
    const GameDatabase::Entry* entry = GameDatabase::GetEntryForSerial(s_running_game_serial);
    if (entry)
      entry->ApplySettings(g_settings, display_osd_messages);
  }

  g_settings.FixIncompatibleSettings(display_osd_messages);
}

void System::SetDefaultSettings(SettingsInterface& si)
{
  Settings temp;

  // we don't want to reset some things (e.g. OSD)
  temp.display_show_osd_messages = g_settings.display_show_osd_messages;
  temp.display_show_fps = g_settings.display_show_fps;
  temp.display_show_speed = g_settings.display_show_speed;
  temp.display_show_resolution = g_settings.display_show_resolution;
  temp.display_show_cpu = g_settings.display_show_cpu;
  temp.display_show_gpu = g_settings.display_show_gpu;
  temp.display_show_frame_times = g_settings.display_show_frame_times;

  // keep controller, we reset it elsewhere
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
    temp.controller_types[i] = g_settings.controller_types[i];

  temp.Save(si);
}

void System::ApplySettings(bool display_osd_messages)
{
  Log_DevPrint("Applying settings...");

  const Settings old_config(std::move(g_settings));
  g_settings = Settings();
  LoadSettings(display_osd_messages);

  // If we've disabled/enabled game settings, we need to reload without it.
  if (g_settings.apply_game_settings != old_config.apply_game_settings)
  {
    UpdateGameSettingsLayer();
    LoadSettings(display_osd_messages);
  }

  CheckForSettingsChanges(old_config);
  Host::CheckForSettingsChanges(old_config);

  if (IsValid())
    ResetPerformanceCounters();
}

bool System::ReloadGameSettings(bool display_osd_messages)
{
  if (!IsValid() || !UpdateGameSettingsLayer())
    return false;

  ApplySettings(display_osd_messages);
  return true;
}

bool System::UpdateGameSettingsLayer()
{
  std::unique_ptr<INISettingsInterface> new_interface;
  if (g_settings.apply_game_settings && !s_running_game_serial.empty())
  {
    std::string filename(GetGameSettingsPath(s_running_game_serial));
    if (FileSystem::FileExists(filename.c_str()))
    {
      Log_InfoPrintf("Loading game settings from '%s'...", filename.c_str());
      new_interface = std::make_unique<INISettingsInterface>(std::move(filename));
      if (!new_interface->Load())
      {
        Log_ErrorPrintf("Failed to parse game settings ini '%s'", new_interface->GetFileName().c_str());
        new_interface.reset();
      }
    }
    else
    {
      Log_InfoPrintf("No game settings found (tried '%s')", filename.c_str());
    }
  }

  std::string input_profile_name;
  bool use_game_settings_for_controller = false;
  if (new_interface)
  {
    new_interface->GetBoolValue("ControllerPorts", "UseGameSettingsForController", &use_game_settings_for_controller);
    if (!use_game_settings_for_controller)
      new_interface->GetStringValue("ControllerPorts", "InputProfileName", &input_profile_name);
  }

  if (!s_game_settings_interface && !new_interface && s_input_profile_name == input_profile_name)
    return false;

  Host::Internal::SetGameSettingsLayer(new_interface.get());
  s_game_settings_interface = std::move(new_interface);

  std::unique_ptr<INISettingsInterface> input_interface;
  if (!use_game_settings_for_controller)
  {
    if (!input_profile_name.empty())
    {
      const std::string filename(GetInputProfilePath(input_profile_name));
      if (FileSystem::FileExists(filename.c_str()))
      {
        Log_InfoPrintf("Loading input profile from '%s'...", filename.c_str());
        input_interface = std::make_unique<INISettingsInterface>(std::move(filename));
        if (!input_interface->Load())
        {
          Log_ErrorPrintf("Failed to parse input profile ini '%s'", input_interface->GetFileName().c_str());
          input_interface.reset();
          input_profile_name = {};
        }
      }
      else
      {
        Log_InfoPrintf("No input profile found (tried '%s')", filename.c_str());
        input_profile_name = {};
      }
    }

    Host::Internal::SetInputSettingsLayer(input_interface ? input_interface.get() :
                                                            Host::Internal::GetBaseSettingsLayer());
  }
  else
  {
    // using game settings for bindings too
    Host::Internal::SetInputSettingsLayer(s_game_settings_interface.get());
  }

  s_input_settings_interface = std::move(input_interface);
  s_input_profile_name = std::move(input_profile_name);
  return true;
}

void System::ResetSystem()
{
  if (!IsValid())
    return;

#ifdef WITH_CHEEVOS
  if (!Achievements::ConfirmSystemReset())
    return;

  if (Achievements::ResetChallengeMode())
    ApplySettings(false);
#endif

  InternalReset();
  ResetPerformanceCounters();
  ResetThrottler();
  Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "System reset."));
}

void System::PauseSystem(bool paused)
{
  if (paused == IsPaused() || !IsValid())
    return;

  SetState(paused ? State::Paused : State::Running);
  SPU::GetOutputStream()->SetPaused(paused);

  if (paused)
  {
    Host::OnSystemPaused();
  }
  else
  {
    Host::OnSystemResumed();
    UpdateDisplaySync();
    ResetPerformanceCounters();
    ResetThrottler();
  }
}

bool System::LoadState(const char* filename)
{
  if (!IsValid())
    return false;

#ifdef WITH_CHEEVOS
  if (Achievements::ChallengeModeActive() &&
      !Achievements::ConfirmChallengeModeDisable(Host::TranslateString("Achievements", "Loading state")))
  {
    return false;
  }
#endif

  Common::Timer load_timer;

  std::unique_ptr<ByteStream> stream = ByteStream::OpenFile(filename, BYTESTREAM_OPEN_READ | BYTESTREAM_OPEN_STREAMED);
  if (!stream)
    return false;

  Log_InfoPrintf("Loading state from '%s'...", filename);

  {
    const std::string display_name(FileSystem::GetDisplayNameFromPath(filename));
    Host::AddIconOSDMessage(
      "load_state", ICON_FA_FOLDER_OPEN,
      fmt::format(Host::TranslateString("OSDMessage", "Loading state from '{}'...").GetCharArray(),
                  Path::GetFileName(display_name)),
      5.0f);
  }

  SaveUndoLoadState();

  if (!DoLoadState(stream.get(), false, true))
  {
    Host::ReportFormattedErrorAsync(
      "Load State Error", Host::TranslateString("OSDMessage", "Loading state from '%s' failed. Resetting."), filename);

    if (m_undo_load_state)
      UndoLoadState();

    return false;
  }

  ResetPerformanceCounters();
  ResetThrottler();
  Host::RenderDisplay(false);
  Log_VerbosePrintf("Loading state took %.2f msec", load_timer.GetTimeMilliseconds());
  return true;
}

bool System::SaveState(const char* filename, bool backup_existing_save)
{
  if (backup_existing_save && FileSystem::FileExists(filename))
  {
    const std::string backup_filename(Path::ReplaceExtension(filename, "bak"));
    if (!FileSystem::RenamePath(filename, backup_filename.c_str()))
      Log_ErrorPrintf("Failed to rename save state backup '%s'", backup_filename.c_str());
  }

  Common::Timer save_timer;

  std::unique_ptr<ByteStream> stream =
    ByteStream::OpenFile(filename, BYTESTREAM_OPEN_CREATE | BYTESTREAM_OPEN_WRITE | BYTESTREAM_OPEN_TRUNCATE |
                                     BYTESTREAM_OPEN_ATOMIC_UPDATE | BYTESTREAM_OPEN_STREAMED);
  if (!stream)
    return false;

  Log_InfoPrintf("Saving state to '%s'...", filename);

  const u32 screenshot_size = 256;
  const bool result = InternalSaveState(stream.get(), screenshot_size,
                                        g_settings.compress_save_states ? SAVE_STATE_HEADER::COMPRESSION_TYPE_ZSTD :
                                                                          SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE);
  if (!result)
  {
    Host::ReportFormattedErrorAsync(Host::TranslateString("OSDMessage", "Save State"),
                                    Host::TranslateString("OSDMessage", "Saving state to '%s' failed."), filename);
    stream->Discard();
  }
  else
  {
    const std::string display_name(FileSystem::GetDisplayNameFromPath(filename));
    Host::AddIconOSDMessage("save_state", ICON_FA_SAVE,
                            fmt::format(Host::TranslateString("OSDMessage", "State saved to '{}'.").GetCharArray(),
                                        Path::GetFileName(display_name)),
                            5.0f);
    stream->Commit();
  }

  Log_VerbosePrintf("Saving state took %.2f msec", save_timer.GetTimeMilliseconds());
  return result;
}

bool System::SaveResumeState()
{
  if (s_running_game_serial.empty())
    return false;

  const std::string path(GetGameSaveStateFileName(s_running_game_serial, -1));
  return SaveState(path.c_str(), false);
}

bool System::BootSystem(SystemBootParameters parameters)
{
  if (!parameters.save_state.empty())
  {
    // loading a state, so pull the media path from the save state to avoid a double change
    std::string state_media(GetMediaPathFromSaveState(parameters.save_state.c_str()));
    if (FileSystem::FileExists(state_media.c_str()))
      parameters.filename = std::move(state_media);
  }

  if (parameters.filename.empty())
    Log_InfoPrintf("Boot Filename: <BIOS/Shell>");
  else
    Log_InfoPrintf("Boot Filename: %s", parameters.filename.c_str());

  Assert(s_state == State::Shutdown);
  s_state = State::Starting;
  s_startup_cancelled.store(false);
  s_region = g_settings.region;
  Host::OnSystemStarting();

  // Load CD image up and detect region.
  Common::Error error;
  std::unique_ptr<CDImage> disc;
  DiscRegion disc_region = DiscRegion::NonPS1;
  std::string exe_boot;
  std::string psf_boot;
  if (!parameters.filename.empty())
  {
    const bool do_exe_boot = IsExeFileName(parameters.filename);
    const bool do_psf_boot = (!do_exe_boot && IsPsfFileName(parameters.filename));
    if (do_exe_boot || do_psf_boot)
    {
      if (s_region == ConsoleRegion::Auto)
      {
        const DiscRegion file_region =
          (do_exe_boot ? GetRegionForExe(parameters.filename.c_str()) : GetRegionForPsf(parameters.filename.c_str()));
        Log_InfoPrintf("EXE/PSF Region: %s", Settings::GetDiscRegionDisplayName(file_region));
        s_region = GetConsoleRegionForDiscRegion(file_region);
        if (do_psf_boot)
          psf_boot = std::move(parameters.filename);
        else
          exe_boot = std::move(parameters.filename);
      }
    }
    else
    {
      Log_InfoPrintf("Loading CD image '%s'...", parameters.filename.c_str());
      disc = CDImage::Open(parameters.filename.c_str(), g_settings.cdrom_load_image_patches, &error);
      if (!disc)
      {
        Host::ReportErrorAsync("Error", fmt::format("Failed to load CD image '{}': {}",
                                                    Path::GetFileName(parameters.filename), error.GetCodeAndMessage()));
        s_state = State::Shutdown;
        Host::OnSystemDestroyed();
        return false;
      }

      if (s_region == ConsoleRegion::Auto)
      {
        disc_region = GetRegionForImage(disc.get());
        if (disc_region != DiscRegion::Other)
        {
          s_region = GetConsoleRegionForDiscRegion(disc_region);
          Log_InfoPrintf("Auto-detected console %s region for '%s' (region %s)",
                         Settings::GetConsoleRegionName(s_region), parameters.filename.c_str(),
                         Settings::GetDiscRegionName(disc_region));
        }
        else
        {
          s_region = ConsoleRegion::NTSC_U;
          Log_WarningPrintf("Could not determine console region for disc region %s. Defaulting to %s.",
                            Settings::GetDiscRegionName(disc_region), Settings::GetConsoleRegionName(s_region));
        }
      }
    }
  }
  else
  {
    // Default to NTSC for BIOS boot.
    if (s_region == ConsoleRegion::Auto)
      s_region = ConsoleRegion::NTSC_U;
  }

  Log_InfoPrintf("Console Region: %s", Settings::GetConsoleRegionDisplayName(s_region));

  // Switch subimage.
  if (disc && parameters.media_playlist_index != 0 && !disc->SwitchSubImage(parameters.media_playlist_index, &error))
  {
    Host::ReportFormattedErrorAsync("Error", "Failed to switch to subimage %u in '%s': %s",
                                    parameters.media_playlist_index, parameters.filename.c_str(),
                                    error.GetCodeAndMessage().GetCharArray());
    s_state = State::Shutdown;
    Host::OnSystemDestroyed();
    return false;
  }

  // Update running game, this will apply settings as well.
  UpdateRunningGame(disc ? disc->GetFileName().c_str() : parameters.filename.c_str(), disc.get(), true);

  if (!parameters.override_exe.empty())
  {
    if (!FileSystem::FileExists(parameters.override_exe.c_str()) || !IsExeFileName(parameters.override_exe))
    {
      Host::ReportFormattedErrorAsync("Error", "File '%s' is not a valid executable to boot.",
                                      parameters.override_exe.c_str());
      s_state = State::Shutdown;
      Host::OnSystemDestroyed();
      return false;
    }

    Log_InfoPrintf("Overriding boot executable: '%s'", parameters.override_exe.c_str());
    exe_boot = std::move(parameters.override_exe);
  }

  // Check for SBI.
  if (!CheckForSBIFile(disc.get()))
  {
    s_state = State::Shutdown;
    ClearRunningGame();
    Host::OnSystemDestroyed();
    return false;
  }

#ifdef WITH_CHEEVOS
  // Check for resuming with hardcore mode.
  if (!parameters.save_state.empty() && Achievements::ChallengeModeActive() &&
      !Achievements::ConfirmChallengeModeDisable(Host::TranslateString("Achievements", "Resuming state")))
  {
    s_state = State::Shutdown;
    ClearRunningGame();
    Host::OnSystemDestroyed();
    return false;
  }
#endif

  // Load BIOS image.
  if (!LoadBIOS())
  {
    s_state = State::Shutdown;
    ClearRunningGame();
    Host::OnSystemDestroyed();
    return false;
  }

  // Component setup.
  if (!Initialize(parameters.force_software_renderer))
  {
    s_state = State::Shutdown;
    ClearRunningGame();
    Host::OnSystemDestroyed();
    return false;
  }

  UpdateControllers();
  UpdateMemoryCardTypes();
  UpdateMultitaps();
  InternalReset();

  // Enable tty by patching bios.
  if (g_settings.bios_patch_tty_enable)
  {
    if (s_bios_image_info && s_bios_image_info->patch_compatible)
      BIOS::PatchBIOSEnableTTY(Bus::g_bios, Bus::BIOS_SIZE);
    else
      Log_ErrorPrintf("Not patching TTY enable, as BIOS is not patch compatible.");
  }

  // Load EXE late after BIOS.
  if (!exe_boot.empty() && !LoadEXE(exe_boot.c_str()))
  {
    Host::ReportFormattedErrorAsync("Error", "Failed to load EXE file '%s'", exe_boot.c_str());
    DestroySystem();
    return false;
  }
  else if (!psf_boot.empty() && !PSFLoader::Load(psf_boot.c_str()))
  {
    Host::ReportFormattedErrorAsync("Error", "Failed to load PSF file '%s'", psf_boot.c_str());
    DestroySystem();
    return false;
  }

  // Insert CD, and apply fastboot patch if enabled.
  if (disc)
    CDROM::InsertMedia(std::move(disc), disc_region);
  if (CDROM::HasMedia() && (parameters.override_fast_boot.has_value() ? parameters.override_fast_boot.value() :
                                                                        g_settings.bios_patch_fast_boot))
  {
    if (s_bios_image_info && s_bios_image_info->patch_compatible)
    {
      // TODO: Fast boot without patches...
      BIOS::PatchBIOSFastBoot(Bus::g_bios, Bus::BIOS_SIZE);
      s_was_fast_booted = true;
    }
    else
    {
      Log_ErrorPrintf("Not patching fast boot, as BIOS is not patch compatible.");
    }
  }

  // Good to go.
  s_state =
    (g_settings.start_paused || parameters.override_start_paused.value_or(false)) ? State::Paused : State::Running;
  UpdateSoftwareCursor();
  SPU::GetOutputStream()->SetPaused(false);
  Host::OnSystemStarted();

  if (s_state == State::Paused)
    Host::OnSystemPaused();
  else
    Host::OnSystemResumed();

  // try to load the state, if it fails, bail out
  if (!parameters.save_state.empty())
  {
    std::unique_ptr<ByteStream> stream =
      ByteStream::OpenFile(parameters.save_state.c_str(), BYTESTREAM_OPEN_READ | BYTESTREAM_OPEN_STREAMED);
    if (!stream)
    {
      Host::ReportErrorAsync(
        Host::TranslateString("System", "Error"),
        fmt::format(Host::TranslateString("System", "Failed to load save state file '{}' for booting.").GetCharArray(),
                    parameters.save_state));
      DestroySystem();
      return false;
    }

    if (!DoLoadState(stream.get(), false, true))
    {
      DestroySystem();
      return false;
    }
  }

  if (parameters.load_image_to_ram || g_settings.cdrom_load_image_to_ram)
    CDROM::PrecacheMedia();

  if (parameters.fast_forward_to_first_frame)
    FastForwardToFirstFrame();

  if (g_settings.audio_dump_on_boot)
    StartDumpingAudio();

  ResetPerformanceCounters();
  if (IsRunning())
    UpdateSpeedLimiterState();

  return true;
}

bool System::Initialize(bool force_software_renderer)
{
  g_ticks_per_second = ScaleTicksToOverclock(MASTER_CLOCK);
  s_max_slice_ticks = ScaleTicksToOverclock(MASTER_CLOCK / 10);
  s_frame_number = 1;
  s_internal_frame_number = 1;

  s_target_speed = g_settings.emulation_speed;
  s_throttle_frequency = 60.0f;
  s_frame_period = 0;
  s_next_frame_time = 0;
  s_turbo_enabled = false;
  s_fast_forward_enabled = false;

  s_average_frame_time_accumulator = 0.0f;
  s_minimum_frame_time_accumulator = 0.0f;
  s_maximum_frame_time_accumulator = 0.0f;

  s_vps = 0.0f;
  s_fps = 0.0f;
  s_speed = 0.0f;
  s_minimum_frame_time = 0.0f;
  s_maximum_frame_time = 0.0f;
  s_average_frame_time = 0.0f;
  s_cpu_thread_usage = 0.0f;
  s_cpu_thread_time = 0.0f;
  s_sw_thread_usage = 0.0f;
  s_sw_thread_time = 0.0f;
  s_average_gpu_time = 0.0f;
  s_accumulated_gpu_time = 0.0f;
  s_gpu_usage = 0.0f;
  s_last_frame_number = 0;
  s_last_internal_frame_number = 0;
  s_last_global_tick_counter = 0;
  s_presents_since_last_update = 0;
  s_last_cpu_time = 0;
  s_fps_timer.Reset();
  s_frame_timer.Reset();
  s_frame_time_history.fill(0.0f);
  s_frame_time_history_pos = 0;

  TimingEvents::Initialize();

  CPU::Initialize();

  if (!Bus::Initialize())
  {
    CPU::Shutdown();
    return false;
  }

  if (!CreateGPU(force_software_renderer ? GPURenderer::Software : g_settings.gpu_renderer))
  {
    Bus::Shutdown();
    CPU::Shutdown();
    return false;
  }

  GTE::UpdateAspectRatio();

  if (g_settings.gpu_pgxp_enable)
    PGXP::Initialize();

  // Was startup cancelled? (e.g. shading compilers took too long and the user closed the application)
  if (IsStartupCancelled())
  {
    g_gpu.reset();
    Host::ReleaseHostDisplay();
    if (g_settings.gpu_pgxp_enable)
      PGXP::Shutdown();
    CPU::Shutdown();
    Bus::Shutdown();
    return false;
  }

  // CPU code cache must happen after GPU, because it might steal our address space.
  CPU::CodeCache::Initialize();

  DMA::Initialize();
  InterruptController::Initialize();

  CDROM::Initialize();
  Pad::Initialize();
  Timers::Initialize();
  SPU::Initialize();
  MDEC::Initialize();
  SIO::Initialize();
  PCDrv::Initialize();

  static constexpr float WARNING_DURATION = 15.0f;

  if (g_settings.cpu_overclock_active)
  {
    Host::AddFormattedOSDMessage(
      WARNING_DURATION,
      Host::TranslateString("OSDMessage", "CPU clock speed is set to %u%% (%u / %u). This may result in instability."),
      g_settings.GetCPUOverclockPercent(), g_settings.cpu_overclock_numerator, g_settings.cpu_overclock_denominator);
  }
  if (g_settings.cdrom_read_speedup > 1)
  {
    Host::AddFormattedOSDMessage(
      WARNING_DURATION,
      Host::TranslateString("OSDMessage",
                            "CD-ROM read speedup set to %ux (effective speed %ux). This may result in instability."),
      g_settings.cdrom_read_speedup, g_settings.cdrom_read_speedup * 2);
  }
  if (g_settings.cdrom_seek_speedup != 1)
  {
    if (g_settings.cdrom_seek_speedup == 0)
    {
      Host::AddOSDMessage(
        Host::TranslateStdString("OSDMessage", "CD-ROM seek speedup set to instant. This may result in instability."),
        WARNING_DURATION);
    }
    else
    {
      Host::AddFormattedOSDMessage(
        WARNING_DURATION,
        Host::TranslateString("OSDMessage", "CD-ROM seek speedup set to %ux. This may result in instability."),
        g_settings.cdrom_seek_speedup);
    }
  }

  s_cpu_thread_handle = Threading::ThreadHandle::GetForCallingThread();

  UpdateThrottlePeriod();
  UpdateMemorySaveStateSettings();
  return true;
}

void System::DestroySystem()
{
  if (s_state == State::Shutdown)
    return;

  SetTimerResolutionIncreased(false);

  s_cpu_thread_usage = {};

  ClearMemorySaveStates();

  g_texture_replacements.Shutdown();

  PCDrv::Shutdown();
  SIO::Shutdown();
  MDEC::Shutdown();
  SPU::Shutdown();
  Timers::Shutdown();
  Pad::Shutdown();
  CDROM::Shutdown();
  if (g_gpu)
    g_gpu->ResetGraphicsAPIState();
  g_gpu.reset();
  InterruptController::Shutdown();
  DMA::Shutdown();
  PGXP::Shutdown();
  CPU::CodeCache::Shutdown();
  Bus::Shutdown();
  CPU::Shutdown();
  TimingEvents::Shutdown();
  ClearRunningGame();

  // Restore present-all-frames behavior.
  if (g_host_display)
  {
    g_host_display->SetDisplayMaxFPS(0.0f);
    UpdateSoftwareCursor();
    Host::ReleaseHostDisplay();
  }

  s_bios_hash = {};
  s_bios_image_info = nullptr;
  s_was_fast_booted = false;

  Host::OnSystemDestroyed();
}

void System::ClearRunningGame()
{
  s_running_game_serial.clear();
  s_running_game_path.clear();
  s_running_game_title.clear();
  s_running_game_hash = 0;
  s_running_unknown_game = false;
  s_cheat_list.reset();
  s_state = State::Shutdown;

  Host::OnGameChanged(s_running_game_path, s_running_game_serial, s_running_game_title);

#ifdef WITH_CHEEVOS
  Achievements::GameChanged(s_running_game_path, nullptr);
#endif
}

bool System::FastForwardToFirstFrame()
{
  // If we're taking more than 60 seconds to load the game, oof..
  static constexpr u32 MAX_FRAMES_TO_SKIP = 30 * 60;
  const u32 current_frame_number = s_frame_number;
  const u32 current_internal_frame_number = s_internal_frame_number;

  SPU::SetAudioOutputMuted(true);
  while (s_internal_frame_number == current_internal_frame_number &&
         (s_frame_number - current_frame_number) <= MAX_FRAMES_TO_SKIP)
  {
    System::RunFrame();
  }
  SPU::SetAudioOutputMuted(false);

  return (s_internal_frame_number != current_internal_frame_number);
}

void System::Execute()
{
  while (System::IsRunning())
  {
    if (s_display_all_frames)
      System::RunFrame();
    else
      System::RunFrames();

    // this can shut us down
    Host::PumpMessagesOnCPUThread();
    if (!IsValid())
      return;

    if (s_frame_step_request)
    {
      s_frame_step_request = false;
      PauseSystem(true);
    }

    const bool skip_present = g_host_display->ShouldSkipDisplayingFrame();
    Host::RenderDisplay(skip_present);
    if (!skip_present && g_host_display->IsGPUTimingEnabled())
    {
      s_accumulated_gpu_time += g_host_display->GetAndResetAccumulatedGPUTime();
      s_presents_since_last_update++;
    }

    if (s_throttler_enabled)
      System::Throttle();

    // Update perf counters *after* throttling, we want to measure from start-of-frame
    // to start-of-frame, not end-of-frame to end-of-frame (will be noisy due to different
    // amounts of computation happening in each frame).
    System::UpdatePerformanceCounters();
  }
}

void System::RecreateSystem()
{
  Assert(!IsShutdown());

  const bool was_paused = System::IsPaused();
  std::unique_ptr<ByteStream> stream = ByteStream::CreateGrowableMemoryStream(nullptr, 8 * 1024);
  if (!System::InternalSaveState(stream.get(), 0, SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE) || !stream->SeekAbsolute(0))
  {
    Host::ReportErrorAsync("Error", "Failed to save state before system recreation. Shutting down.");
    DestroySystem();
    return;
  }

  DestroySystem();

  SystemBootParameters boot_params;
  if (!BootSystem(std::move(boot_params)))
  {
    Host::ReportErrorAsync("Error", "Failed to boot system after recreation.");
    return;
  }

  if (!DoLoadState(stream.get(), false, false))
  {
    DestroySystem();
    return;
  }

  ResetPerformanceCounters();
  ResetThrottler();
  Host::RenderDisplay(false);

  if (was_paused)
    PauseSystem(true);
}

bool System::CreateGPU(GPURenderer renderer)
{
  switch (renderer)
  {
#ifdef WITH_OPENGL
    case GPURenderer::HardwareOpenGL:
      g_gpu = GPU::CreateHardwareOpenGLRenderer();
      break;
#endif

#ifdef WITH_VULKAN
    case GPURenderer::HardwareVulkan:
      g_gpu = GPU::CreateHardwareVulkanRenderer();
      break;
#endif

#ifdef _WIN32
    case GPURenderer::HardwareD3D11:
      g_gpu = GPU::CreateHardwareD3D11Renderer();
      break;
    case GPURenderer::HardwareD3D12:
      g_gpu = GPU::CreateHardwareD3D12Renderer();
      break;
#endif

    case GPURenderer::Software:
    default:
      g_gpu = GPU::CreateSoftwareRenderer();
      break;
  }

  if (!g_gpu)
  {
    Log_ErrorPrintf("Failed to initialize %s renderer, falling back to software renderer",
                    Settings::GetRendererName(renderer));
    Host::AddFormattedOSDMessage(
      30.0f,
      Host::TranslateString("OSDMessage", "Failed to initialize %s renderer, falling back to software renderer."),
      Settings::GetRendererName(renderer));
    g_gpu.reset();
    g_gpu = GPU::CreateSoftwareRenderer();
    if (!g_gpu)
    {
      Log_ErrorPrintf("Failed to create fallback software renderer.");
      return false;
    }
  }

  return true;
}

bool System::DoState(StateWrapper& sw, GPUTexture** host_texture, bool update_display, bool is_memory_state)
{
  if (!sw.DoMarker("System"))
    return false;

  sw.Do(&s_region);
  sw.Do(&s_frame_number);
  sw.Do(&s_internal_frame_number);

  // Don't bother checking this at all for memory states, since they won't have a different BIOS...
  if (!is_memory_state)
  {
    BIOS::Hash bios_hash = s_bios_hash;
    sw.DoBytesEx(bios_hash.bytes, sizeof(bios_hash.bytes), 58, s_bios_hash.bytes);
    if (bios_hash != s_bios_hash)
    {
      Log_WarningPrintf("BIOS hash mismatch: System: %s | State: %s", s_bios_hash.ToString().c_str(),
                        bios_hash.ToString().c_str());
      Host::AddKeyedOSDMessage(
        "StateBIOSMismatch",
        Host::TranslateStdString("OSDMessage", "This save state was created with a different BIOS version or patch "
                                               "options. This may cause stability issues."),
        10.0f);
    }
  }

  if (!sw.DoMarker("CPU") || !CPU::DoState(sw))
    return false;

  if (sw.IsReading())
  {
    if (is_memory_state)
      CPU::CodeCache::InvalidateAll();
    else
      CPU::CodeCache::Flush();
  }

  // only reset pgxp if we're not runahead-rollbacking. the value checks will save us from broken rendering, and it
  // saves using imprecise values for a frame in 30fps games.
  if (sw.IsReading() && g_settings.gpu_pgxp_enable && !is_memory_state)
    PGXP::Reset();

  if (!sw.DoMarker("Bus") || !Bus::DoState(sw))
    return false;

  if (!sw.DoMarker("DMA") || !DMA::DoState(sw))
    return false;

  if (!sw.DoMarker("InterruptController") || !InterruptController::DoState(sw))
    return false;

  g_gpu->RestoreGraphicsAPIState();
  const bool gpu_result = sw.DoMarker("GPU") && g_gpu->DoState(sw, host_texture, update_display);
  g_gpu->ResetGraphicsAPIState();
  if (!gpu_result)
    return false;

  if (!sw.DoMarker("CDROM") || !CDROM::DoState(sw))
    return false;

  if (!sw.DoMarker("Pad") || !Pad::DoState(sw, is_memory_state))
    return false;

  if (!sw.DoMarker("Timers") || !Timers::DoState(sw))
    return false;

  if (!sw.DoMarker("SPU") || !SPU::DoState(sw))
    return false;

  if (!sw.DoMarker("MDEC") || !MDEC::DoState(sw))
    return false;

  if (!sw.DoMarker("SIO") || !SIO::DoState(sw))
    return false;

  if (!sw.DoMarker("Events") || !TimingEvents::DoState(sw))
    return false;

  if (!sw.DoMarker("Overclock"))
    return false;

  bool cpu_overclock_active = g_settings.cpu_overclock_active;
  u32 cpu_overclock_numerator = g_settings.cpu_overclock_numerator;
  u32 cpu_overclock_denominator = g_settings.cpu_overclock_denominator;
  sw.Do(&cpu_overclock_active);
  sw.Do(&cpu_overclock_numerator);
  sw.Do(&cpu_overclock_denominator);

  if (sw.IsReading() && (cpu_overclock_active != g_settings.cpu_overclock_active ||
                         (cpu_overclock_active && (g_settings.cpu_overclock_numerator != cpu_overclock_numerator ||
                                                   g_settings.cpu_overclock_denominator != cpu_overclock_denominator))))
  {
    Host::AddFormattedOSDMessage(
      10.0f, Host::TranslateString("OSDMessage", "WARNING: CPU overclock (%u%%) was different in save state (%u%%)."),
      g_settings.cpu_overclock_enable ? g_settings.GetCPUOverclockPercent() : 100u,
      cpu_overclock_active ?
        Settings::CPUOverclockFractionToPercent(cpu_overclock_numerator, cpu_overclock_denominator) :
        100u);
    UpdateOverclock();
  }

  if (!is_memory_state)
  {
    if (sw.GetVersion() >= 56)
    {
      if (!sw.DoMarker("Cheevos"))
        return false;

#ifdef WITH_CHEEVOS
      if (!Achievements::DoState(sw))
        return false;
#else
      // if we compiled without cheevos, we need to toss out the data from states which were
      u32 data_size = 0;
      sw.Do(&data_size);
      if (data_size > 0)
        sw.SkipBytes(data_size);
#endif
    }
    else
    {
#ifdef WITH_CHEEVOS
      // loading an old state without cheevos, so reset the runtime
      Achievements::ResetRuntime();
#endif
    }
  }

  return !sw.HasError();
}

bool System::LoadBIOS()
{
  std::optional<BIOS::Image> bios_image(BIOS::GetBIOSImage(s_region));
  if (!bios_image.has_value())
  {
    Host::ReportFormattedErrorAsync("Error", Host::TranslateString("System", "Failed to load %s BIOS."),
                                    Settings::GetConsoleRegionName(s_region));
    return false;
  }

  if (bios_image->size() != static_cast<u32>(Bus::BIOS_SIZE))
  {
    Host::ReportFormattedErrorAsync("Error", Host::TranslateString("System", "Incorrect BIOS image size"));
    return false;
  }

  s_bios_hash = BIOS::GetImageHash(bios_image.value());
  s_bios_image_info = BIOS::GetInfoForImage(bios_image.value(), s_bios_hash);
  if (s_bios_image_info)
    Log_InfoPrintf("Using BIOS: %s", s_bios_image_info->description);
  else
    Log_WarningPrintf("Using an unknown BIOS: %s", s_bios_hash.ToString().c_str());

  std::memcpy(Bus::g_bios, bios_image->data(), Bus::BIOS_SIZE);
  return true;
}

void System::InternalReset()
{
  if (IsShutdown())
    return;

  g_gpu->RestoreGraphicsAPIState();

  CPU::Reset();
  CPU::CodeCache::Flush();
  if (g_settings.gpu_pgxp_enable)
    PGXP::Initialize();

  Bus::Reset();
  DMA::Reset();
  InterruptController::Reset();
  g_gpu->Reset(true);
  CDROM::Reset();
  Pad::Reset();
  Timers::Reset();
  SPU::Reset();
  MDEC::Reset();
  SIO::Reset();
  PCDrv::Reset();
  s_frame_number = 1;
  s_internal_frame_number = 0;
  TimingEvents::Reset();
  ResetPerformanceCounters();

#ifdef WITH_CHEEVOS
  Achievements::ResetRuntime();
#endif

  g_gpu->ResetGraphicsAPIState();
}

std::string System::GetMediaPathFromSaveState(const char* path)
{
  std::string ret;

  std::unique_ptr<ByteStream> stream(ByteStream::OpenFile(path, BYTESTREAM_OPEN_READ | BYTESTREAM_OPEN_SEEKABLE));
  if (stream)
  {
    SAVE_STATE_HEADER header;
    if (stream->Read2(&header, sizeof(header)) && header.magic == SAVE_STATE_MAGIC &&
        header.version >= SAVE_STATE_MINIMUM_VERSION && header.version <= SAVE_STATE_VERSION)
    {
      if (header.media_filename_length > 0)
      {
        ret.resize(header.media_filename_length);
        if (!stream->SeekAbsolute(header.offset_to_media_filename) ||
            !stream->Read2(ret.data(), header.media_filename_length))
        {
          ret = {};
        }
      }
    }
  }

  return ret;
}

bool System::DoLoadState(ByteStream* state, bool force_software_renderer, bool update_display)
{
  Assert(IsValid());

  SAVE_STATE_HEADER header;
  if (!state->Read2(&header, sizeof(header)))
    return false;

  if (header.magic != SAVE_STATE_MAGIC)
    return false;

  if (header.version < SAVE_STATE_MINIMUM_VERSION)
  {
    Host::ReportFormattedErrorAsync(
      "Error",
      Host::TranslateString("System", "Save state is incompatible: minimum version is %u but state is version %u."),
      SAVE_STATE_MINIMUM_VERSION, header.version);
    return false;
  }

  if (header.version > SAVE_STATE_VERSION)
  {
    Host::ReportFormattedErrorAsync(
      "Error",
      Host::TranslateString("System", "Save state is incompatible: maximum version is %u but state is version %u."),
      SAVE_STATE_VERSION, header.version);
    return false;
  }

  Common::Error error;
  std::string media_filename;
  std::unique_ptr<CDImage> media;
  if (header.media_filename_length > 0)
  {
    media_filename.resize(header.media_filename_length);
    if (!state->SeekAbsolute(header.offset_to_media_filename) ||
        !state->Read2(media_filename.data(), header.media_filename_length))
    {
      return false;
    }

    std::unique_ptr<CDImage> old_media = CDROM::RemoveMedia(false);
    if (old_media && old_media->GetFileName() == media_filename)
    {
      Log_InfoPrintf("Re-using same media '%s'", media_filename.c_str());
      media = std::move(old_media);
    }
    else
    {
      media = CDImage::Open(media_filename.c_str(), g_settings.cdrom_load_image_patches, &error);
      if (!media)
      {
        if (old_media)
        {
          Host::AddFormattedOSDMessage(
            30.0f,
            Host::TranslateString("OSDMessage", "Failed to open CD image from save state '%s': %s. Using "
                                                "existing image '%s', this may result in instability."),
            media_filename.c_str(), error.GetCodeAndMessage().GetCharArray(), old_media->GetFileName().c_str());
          media = std::move(old_media);
          header.media_subimage_index = media->GetCurrentSubImage();
        }
        else
        {
          Host::ReportFormattedErrorAsync(
            "Error", Host::TranslateString("System", "Failed to open CD image '%s' used by save state: %s."),
            media_filename.c_str(), error.GetCodeAndMessage().GetCharArray());
          return false;
        }
      }
    }
  }

  UpdateRunningGame(media_filename.c_str(), media.get(), false);

  if (media && header.version >= 51)
  {
    const u32 num_subimages = media->HasSubImages() ? media->GetSubImageCount() : 1;
    if (header.media_subimage_index >= num_subimages ||
        (media->HasSubImages() && media->GetCurrentSubImage() != header.media_subimage_index &&
         !media->SwitchSubImage(header.media_subimage_index, &error)))
    {
      Host::ReportFormattedErrorAsync(
        "Error",
        Host::TranslateString("System", "Failed to switch to subimage %u in CD image '%s' used by save state: %s."),
        header.media_subimage_index + 1u, media_filename.c_str(), error.GetCodeAndMessage().GetCharArray());
      return false;
    }
    else
    {
      Log_InfoPrintf("Switched to subimage %u in '%s'", header.media_subimage_index, media_filename.c_str());
    }
  }

  ClearMemorySaveStates();

  CDROM::Reset();
  if (media)
  {
    const DiscRegion region = GetRegionForImage(media.get());
    CDROM::InsertMedia(std::move(media), region);
    if (g_settings.cdrom_load_image_to_ram)
      CDROM::PrecacheMedia();
  }
  else
  {
    CDROM::RemoveMedia(false);
  }

  // ensure the correct card is loaded
  if (g_settings.HasAnyPerGameMemoryCards())
    UpdatePerGameMemoryCards();

#ifdef WITH_CHEEVOS
  // Updating game/loading settings can turn on hardcore mode. Catch this.
  if (Achievements::ChallengeModeActive())
  {
    Host::AddKeyedOSDMessage("challenge_mode_reset",
                             Host::TranslateStdString("Achievements", "Hardcore mode disabled by state switch."),
                             10.0f);
    Achievements::DisableChallengeMode();
  }
#endif

  if (!state->SeekAbsolute(header.offset_to_data))
    return false;

  if (header.data_compression_type == SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE)
  {
    StateWrapper sw(state, StateWrapper::Mode::Read, header.version);
    if (!DoState(sw, nullptr, update_display, false))
      return false;
  }
  else if (header.data_compression_type == SAVE_STATE_HEADER::COMPRESSION_TYPE_ZSTD)
  {
    std::unique_ptr<ByteStream> dstream(ByteStream::CreateZstdDecompressStream(state, header.data_compressed_size));
    StateWrapper sw(dstream.get(), StateWrapper::Mode::Read, header.version);
    if (!DoState(sw, nullptr, update_display, false))
      return false;
  }
  else
  {
    Host::ReportFormattedErrorAsync("Error", "Unknown save state compression type %u", header.data_compression_type);
    return false;
  }

  if (s_state == State::Starting)
    s_state = State::Running;

  ResetPerformanceCounters();
  ResetThrottler();
  return true;
}

bool System::InternalSaveState(ByteStream* state, u32 screenshot_size /* = 256 */,
                               u32 compression_method /* = SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE*/)
{
  if (IsShutdown())
    return false;

  SAVE_STATE_HEADER header = {};

  const u64 header_position = state->GetPosition();
  if (!state->Write2(&header, sizeof(header)))
    return false;

  // fill in header
  header.magic = SAVE_STATE_MAGIC;
  header.version = SAVE_STATE_VERSION;
  StringUtil::Strlcpy(header.title, s_running_game_title.c_str(), sizeof(header.title));
  StringUtil::Strlcpy(header.serial, s_running_game_serial.c_str(), sizeof(header.serial));

  if (CDROM::HasMedia())
  {
    const std::string& media_filename = CDROM::GetMediaFileName();
    header.offset_to_media_filename = static_cast<u32>(state->GetPosition());
    header.media_filename_length = static_cast<u32>(media_filename.length());
    header.media_subimage_index = CDROM::GetMedia()->HasSubImages() ? CDROM::GetMedia()->GetCurrentSubImage() : 0;
    if (!media_filename.empty() && !state->Write2(media_filename.data(), header.media_filename_length))
      return false;
  }

  // save screenshot
  if (screenshot_size > 0)
  {
    // assume this size is the width
    const float display_aspect_ratio = g_host_display->GetDisplayAspectRatio();
    const u32 screenshot_width = screenshot_size;
    const u32 screenshot_height =
      std::max(1u, static_cast<u32>(static_cast<float>(screenshot_width) /
                                    ((display_aspect_ratio > 0.0f) ? display_aspect_ratio : 1.0f)));
    Log_VerbosePrintf("Saving %ux%u screenshot for state", screenshot_width, screenshot_height);

    std::vector<u32> screenshot_buffer;
    u32 screenshot_stride;
    GPUTexture::Format screenshot_format;
    if (g_host_display->RenderScreenshot(screenshot_width, screenshot_height,
                                         Common::Rectangle<s32>::FromExtents(0, 0, screenshot_width, screenshot_height),
                                         &screenshot_buffer, &screenshot_stride, &screenshot_format) &&
        GPUTexture::ConvertTextureDataToRGBA8(screenshot_width, screenshot_height, screenshot_buffer, screenshot_stride,
                                              screenshot_format))
    {
      if (screenshot_stride != (screenshot_width * sizeof(u32)))
      {
        Log_WarningPrintf("Failed to save %ux%u screenshot for save state due to incorrect stride(%u)",
                          screenshot_width, screenshot_height, screenshot_stride);
      }
      else
      {
        if (g_host_display->UsesLowerLeftOrigin())
        {
          GPUTexture::FlipTextureDataRGBA8(screenshot_width, screenshot_height, screenshot_buffer, screenshot_stride);
        }

        header.offset_to_screenshot = static_cast<u32>(state->GetPosition());
        header.screenshot_width = screenshot_width;
        header.screenshot_height = screenshot_height;
        header.screenshot_size = static_cast<u32>(screenshot_buffer.size() * sizeof(u32));
        if (!state->Write2(screenshot_buffer.data(), header.screenshot_size))
          return false;
      }
    }
    else
    {
      Log_WarningPrintf("Failed to save %ux%u screenshot for save state due to render/conversion failure",
                        screenshot_width, screenshot_height);
    }
  }

  // write data
  {
    header.offset_to_data = static_cast<u32>(state->GetPosition());

    g_gpu->RestoreGraphicsAPIState();

    header.data_compression_type = compression_method;

    bool result = false;
    if (compression_method == SAVE_STATE_HEADER::COMPRESSION_TYPE_NONE)
    {
      StateWrapper sw(state, StateWrapper::Mode::Write, SAVE_STATE_VERSION);
      result = DoState(sw, nullptr, false, false);
      header.data_uncompressed_size = static_cast<u32>(state->GetPosition() - header.offset_to_data);
    }
    else if (compression_method == SAVE_STATE_HEADER::COMPRESSION_TYPE_ZSTD)
    {
      std::unique_ptr<ByteStream> cstream(ByteStream::CreateZstdCompressStream(state, 0));
      StateWrapper sw(cstream.get(), StateWrapper::Mode::Write, SAVE_STATE_VERSION);
      result = DoState(sw, nullptr, false, false) && cstream->Commit();
      header.data_uncompressed_size = static_cast<u32>(cstream->GetPosition());
      header.data_compressed_size = static_cast<u32>(state->GetPosition() - header.offset_to_data);
    }

    g_gpu->ResetGraphicsAPIState();

    if (!result)
      return false;
  }

  // re-write header
  const u64 end_position = state->GetPosition();
  if (!state->SeekAbsolute(header_position) || !state->Write2(&header, sizeof(header)) ||
      !state->SeekAbsolute(end_position))
  {
    return false;
  }

  return true;
}

void System::SingleStepCPU()
{
  const u32 old_frame_number = s_frame_number;

  s_frame_timer.Reset();

  g_gpu->RestoreGraphicsAPIState();

  CPU::SingleStep();

  SPU::GeneratePendingSamples();

  if (s_frame_number != old_frame_number && s_cheat_list)
    s_cheat_list->Apply();

  g_gpu->ResetGraphicsAPIState();
}

void System::DoRunFrame()
{
  g_gpu->RestoreGraphicsAPIState();

  if (CPU::g_state.use_debug_dispatcher)
  {
    CPU::ExecuteDebug();
  }
  else
  {
    switch (g_settings.cpu_execution_mode)
    {
      case CPUExecutionMode::Recompiler:
#ifdef WITH_RECOMPILER
        CPU::CodeCache::ExecuteRecompiler();
#else
        CPU::CodeCache::Execute();
#endif
        break;

      case CPUExecutionMode::CachedInterpreter:
        CPU::CodeCache::Execute();
        break;

      case CPUExecutionMode::Interpreter:
      default:
        CPU::Execute();
        break;
    }
  }

  // Generate any pending samples from the SPU before sleeping, this way we reduce the chances of underruns.
  SPU::GeneratePendingSamples();

  if (s_cheat_list)
    s_cheat_list->Apply();

  g_gpu->ResetGraphicsAPIState();
}

void System::RunFrame()
{
  if (s_rewind_load_counter >= 0)
  {
    DoRewind();
    return;
  }

  if (s_runahead_frames > 0)
    DoRunahead();

  DoRunFrame();

  s_next_frame_time += s_frame_period;

  if (s_memory_saves_enabled)
    DoMemorySaveStates();
}

float System::GetTargetSpeed()
{
  return s_target_speed;
}

void System::SetThrottleFrequency(float frequency)
{
  s_throttle_frequency = frequency;
  UpdateThrottlePeriod();
}

void System::UpdateThrottlePeriod()
{
  if (s_target_speed > std::numeric_limits<double>::epsilon())
  {
    const double target_speed = std::max(static_cast<double>(s_target_speed), std::numeric_limits<double>::epsilon());
    s_frame_period =
      Common::Timer::ConvertSecondsToValue(1.0 / (static_cast<double>(s_throttle_frequency) * target_speed));
  }
  else
  {
    s_frame_period = 1;
  }

  ResetThrottler();
}

void System::ResetThrottler()
{
  s_next_frame_time = Common::Timer::GetCurrentValue();
}

void System::Throttle()
{
  // If we're running too slow, advance the next frame time based on the time we lost. Effectively skips
  // running those frames at the intended time, because otherwise if we pause in the debugger, we'll run
  // hundreds of frames when we resume.
  Common::Timer::Value current_time = Common::Timer::GetCurrentValue();
  if (current_time > s_next_frame_time)
  {
    const Common::Timer::Value diff = static_cast<s64>(current_time) - static_cast<s64>(s_next_frame_time);
    s_next_frame_time += (diff / s_frame_period) * s_frame_period;
    return;
  }

  // Use a spinwait if we undersleep for all platforms except android.. don't want to burn battery.
  // Linux also seems to do a much better job of waking up at the requested time.
#if !defined(__linux__) && !defined(__ANDROID__)
  Common::Timer::SleepUntil(s_next_frame_time, g_settings.display_all_frames);
#else
  Common::Timer::SleepUntil(s_next_frame_time, false);
#endif
}

void System::RunFrames()
{
  // If we're running more than this in a single loop... we're in for a bad time.
  const u32 max_frames_to_run = 2;
  u32 frames_run = 0;

  Common::Timer::Value value = Common::Timer::GetCurrentValue();
  while (frames_run < max_frames_to_run)
  {
    if (value < s_next_frame_time)
      break;

    RunFrame();
    frames_run++;

    value = Common::Timer::GetCurrentValue();
  }

  if (frames_run != 1)
    Log_VerbosePrintf("Ran %u frames in a single host frame", frames_run);
}

void System::UpdatePerformanceCounters()
{
  const float frame_time = static_cast<float>(s_frame_timer.GetTimeMillisecondsAndReset());
  s_minimum_frame_time_accumulator =
    (s_minimum_frame_time_accumulator == 0.0f) ? frame_time : std::min(s_minimum_frame_time_accumulator, frame_time);
  s_average_frame_time_accumulator += frame_time;
  s_maximum_frame_time_accumulator = std::max(s_maximum_frame_time_accumulator, frame_time);
  s_frame_time_history[s_frame_time_history_pos] = frame_time;
  s_frame_time_history_pos = (s_frame_time_history_pos + 1) % NUM_FRAME_TIME_SAMPLES;

  // update fps counter
  const Common::Timer::Value now_ticks = Common::Timer::GetCurrentValue();
  const Common::Timer::Value ticks_diff = now_ticks - s_fps_timer.GetStartValue();
  const float time = static_cast<float>(Common::Timer::ConvertValueToSeconds(ticks_diff));
  if (time < PERFORMANCE_COUNTER_UPDATE_INTERVAL)
    return;

  const float frames_run = static_cast<float>(s_frame_number - s_last_frame_number);
  const u32 global_tick_counter = TimingEvents::GetGlobalTickCounter();

  // TODO: Make the math here less rubbish
  const double pct_divider =
    100.0 * (1.0 / ((static_cast<double>(ticks_diff) * static_cast<double>(Threading::GetThreadTicksPerSecond())) /
                    Common::Timer::GetFrequency() / 1000000000.0));
  const double time_divider = 1000.0 * (1.0 / static_cast<double>(Threading::GetThreadTicksPerSecond())) *
                              (1.0 / static_cast<double>(frames_run));

  s_minimum_frame_time = std::exchange(s_minimum_frame_time_accumulator, 0.0f);
  s_average_frame_time = std::exchange(s_average_frame_time_accumulator, 0.0f) / frames_run;
  s_maximum_frame_time = std::exchange(s_maximum_frame_time_accumulator, 0.0f);

  s_vps = static_cast<float>(frames_run / time);
  s_last_frame_number = s_frame_number;
  s_fps = static_cast<float>(s_internal_frame_number - s_last_internal_frame_number) / time;
  s_last_internal_frame_number = s_internal_frame_number;
  s_speed = static_cast<float>(static_cast<double>(global_tick_counter - s_last_global_tick_counter) /
                               (static_cast<double>(g_ticks_per_second) * time)) *
            100.0f;
  s_last_global_tick_counter = global_tick_counter;

  const Threading::Thread* sw_thread = g_gpu->GetSWThread();
  const u64 cpu_time = s_cpu_thread_handle ? s_cpu_thread_handle.GetCPUTime() : 0;
  const u64 sw_time = sw_thread ? sw_thread->GetCPUTime() : 0;
  const u64 cpu_delta = cpu_time - s_last_cpu_time;
  const u64 sw_delta = sw_time - s_last_sw_time;
  s_last_cpu_time = cpu_time;
  s_last_sw_time = sw_time;

  s_cpu_thread_usage = static_cast<float>(static_cast<double>(cpu_delta) * pct_divider);
  s_cpu_thread_time = static_cast<float>(static_cast<double>(cpu_delta) * time_divider);
  s_sw_thread_usage = static_cast<float>(static_cast<double>(sw_delta) * pct_divider);
  s_sw_thread_time = static_cast<float>(static_cast<double>(sw_delta) * time_divider);

  s_fps_timer.ResetTo(now_ticks);

  if (g_host_display->IsGPUTimingEnabled())
  {
    s_average_gpu_time = s_accumulated_gpu_time / static_cast<float>(std::max(s_presents_since_last_update, 1u));
    s_gpu_usage = s_accumulated_gpu_time / (time * 10.0f);
  }
  s_accumulated_gpu_time = 0.0f;
  s_presents_since_last_update = 0;

  Log_VerbosePrintf("FPS: %.2f VPS: %.2f CPU: %.2f GPU: %.2f Average: %.2fms Min: %.2fms Max: %.2f ms", s_fps, s_vps,
                    s_cpu_thread_usage, s_gpu_usage, s_average_frame_time, s_minimum_frame_time, s_maximum_frame_time);

  Host::OnPerformanceCountersUpdated();
}

void System::ResetPerformanceCounters()
{
  s_last_frame_number = s_frame_number;
  s_last_internal_frame_number = s_internal_frame_number;
  s_last_global_tick_counter = TimingEvents::GetGlobalTickCounter();
  s_last_cpu_time = s_cpu_thread_handle ? s_cpu_thread_handle.GetCPUTime() : 0;
  if (const Threading::Thread* sw_thread = g_gpu->GetSWThread(); sw_thread)
    s_last_sw_time = sw_thread->GetCPUTime();
  else
    s_last_sw_time = 0;

  s_average_frame_time_accumulator = 0.0f;
  s_minimum_frame_time_accumulator = 0.0f;
  s_maximum_frame_time_accumulator = 0.0f;
  s_frame_timer.Reset();
  s_fps_timer.Reset();
  ResetThrottler();
}

void System::UpdateSpeedLimiterState()
{
  const float old_target_speed = s_target_speed;
  s_target_speed = s_turbo_enabled ?
                     g_settings.turbo_speed :
                     (s_fast_forward_enabled ? g_settings.fast_forward_speed : g_settings.emulation_speed);
  s_throttler_enabled = (s_target_speed != 0.0f);
  s_display_all_frames = !s_throttler_enabled || g_settings.display_all_frames;

  s_syncing_to_host = false;
  if (g_settings.sync_to_host_refresh_rate && (g_settings.audio_stretch_mode != AudioStretchMode::Off) &&
      s_target_speed == 1.0f && IsValid())
  {
    float host_refresh_rate;
    if (g_host_display->GetHostRefreshRate(&host_refresh_rate))
    {
      const float ratio = host_refresh_rate / System::GetThrottleFrequency();
      s_syncing_to_host = (ratio >= 0.95f && ratio <= 1.05f);
      Log_InfoPrintf("Refresh rate: Host=%fhz Guest=%fhz Ratio=%f - %s", host_refresh_rate,
                     System::GetThrottleFrequency(), ratio, s_syncing_to_host ? "can sync" : "can't sync");
      if (s_syncing_to_host)
        s_target_speed *= ratio;
    }
  }

  // When syncing to host and using vsync, we don't need to sleep.
  if (s_syncing_to_host && ShouldUseVSync() && s_display_all_frames)
  {
    Log_InfoPrintf("Using host vsync for throttling.");
    s_throttler_enabled = false;
  }

  Log_VerbosePrintf("Target speed: %f%%", s_target_speed * 100.0f);

  if (IsValid())
  {
    // Update audio output.
    AudioStream* stream = SPU::GetOutputStream();
    stream->SetOutputVolume(GetAudioOutputVolume());

    // Adjust nominal rate when resampling, or syncing to host.
    const bool rate_adjust =
      (s_syncing_to_host || g_settings.audio_stretch_mode == AudioStretchMode::Resample) && s_target_speed > 0.0f;
    stream->SetNominalRate(rate_adjust ? s_target_speed : 1.0f);

    if (old_target_speed < s_target_speed)
      stream->UpdateTargetTempo(s_target_speed);

    UpdateThrottlePeriod();
    ResetThrottler();
  }

  // Defer vsync update until we unpause, in case of fullscreen UI.
  if (IsRunning())
    UpdateDisplaySync();

  if (g_settings.increase_timer_resolution)
    SetTimerResolutionIncreased(s_throttler_enabled);
}

void System::UpdateDisplaySync()
{
  const bool video_sync_enabled = ShouldUseVSync();
  const bool syncing_to_host_vsync = (s_syncing_to_host && video_sync_enabled && s_display_all_frames);
  const float max_display_fps = (s_throttler_enabled || s_syncing_to_host) ? 0.0f : g_settings.display_max_fps;
  Log_VerbosePrintf("Using vsync: %s%s", video_sync_enabled ? "YES" : "NO",
                    syncing_to_host_vsync ? " (for throttling)" : "");
  Log_VerbosePrintf("Max display fps: %f (%s)", max_display_fps,
                    s_display_all_frames ? "displaying all frames" : "skipping displaying frames when needed");

  g_host_display->SetDisplayMaxFPS(max_display_fps);
  g_host_display->SetVSync(video_sync_enabled);
}

bool System::ShouldUseVSync()
{
  return g_settings.video_sync_enabled && !IsRunningAtNonStandardSpeed();
}

bool System::IsFastForwardEnabled()
{
  return s_fast_forward_enabled;
}

void System::SetFastForwardEnabled(bool enabled)
{
  if (!IsValid())
    return;

  s_fast_forward_enabled = enabled;
  UpdateSpeedLimiterState();
}

bool System::IsTurboEnabled()
{
  return s_turbo_enabled;
}

void System::SetTurboEnabled(bool enabled)
{
  if (!IsValid())
    return;

  s_turbo_enabled = enabled;
  UpdateSpeedLimiterState();
}

void System::SetRewindState(bool enabled)
{
  if (!System::IsValid())
    return;

  if (!g_settings.rewind_enable)
  {
    if (enabled)
      Host::AddKeyedOSDMessage("SetRewindState", Host::TranslateStdString("OSDMessage", "Rewinding is not enabled."),
                               5.0f);

    return;
  }

#ifdef WITH_CHEEVOS
  if (Achievements::ChallengeModeActive() && !Achievements::ConfirmChallengeModeDisable("Rewinding"))
    return;
#endif

  System::SetRewinding(enabled);
  UpdateSpeedLimiterState();
}

void System::DoFrameStep()
{
  if (!IsValid())
    return;

#ifdef WITH_CHEEVOS
  if (Achievements::ChallengeModeActive() && !Achievements::ConfirmChallengeModeDisable("Frame stepping"))
    return;
#endif

  s_frame_step_request = true;
  PauseSystem(false);
}

void System::DoToggleCheats()
{
  if (!System::IsValid())
    return;

#ifdef WITH_CHEEVOS
  if (Achievements::ChallengeModeActive() && !Achievements::ConfirmChallengeModeDisable("Toggling cheats"))
    return;
#endif

  CheatList* cl = GetCheatList();
  if (!cl)
  {
    Host::AddKeyedOSDMessage("ToggleCheats", Host::TranslateStdString("OSDMessage", "No cheats are loaded."), 10.0f);
    return;
  }

  cl->SetMasterEnable(!cl->GetMasterEnable());
  Host::AddKeyedOSDMessage(
    "ToggleCheats",
    cl->GetMasterEnable() ?
      Host::TranslateStdString("OSDMessage", "%n cheats are now active.", "", cl->GetEnabledCodeCount()) :
      Host::TranslateStdString("OSDMessage", "%n cheats are now inactive.", "", cl->GetEnabledCodeCount()),
    10.0f);
}

static bool LoadEXEToRAM(const char* filename, bool patch_bios)
{
  std::FILE* fp = FileSystem::OpenCFile(filename, "rb");
  if (!fp)
  {
    Log_ErrorPrintf("Failed to open exe file '%s'", filename);
    return false;
  }

  std::fseek(fp, 0, SEEK_END);
  const u32 file_size = static_cast<u32>(std::ftell(fp));
  std::fseek(fp, 0, SEEK_SET);

  BIOS::PSEXEHeader header;
  if (std::fread(&header, sizeof(header), 1, fp) != 1 || !BIOS::IsValidPSExeHeader(header, file_size))
  {
    Log_ErrorPrintf("'%s' is not a valid PS-EXE", filename);
    std::fclose(fp);
    return false;
  }

  if (header.memfill_size > 0)
  {
    const u32 words_to_write = header.memfill_size / 4;
    u32 address = header.memfill_start & ~UINT32_C(3);
    for (u32 i = 0; i < words_to_write; i++)
    {
      CPU::SafeWriteMemoryWord(address, 0);
      address += sizeof(u32);
    }
  }

  const u32 file_data_size = std::min<u32>(file_size - sizeof(BIOS::PSEXEHeader), header.file_size);
  if (file_data_size >= 4)
  {
    std::vector<u32> data_words((file_data_size + 3) / 4);
    if (std::fread(data_words.data(), file_data_size, 1, fp) != 1)
    {
      std::fclose(fp);
      return false;
    }

    const u32 num_words = file_data_size / 4;
    u32 address = header.load_address;
    for (u32 i = 0; i < num_words; i++)
    {
      CPU::SafeWriteMemoryWord(address, data_words[i]);
      address += sizeof(u32);
    }
  }

  std::fclose(fp);

  // patch the BIOS to jump to the executable directly
  const u32 r_pc = header.initial_pc;
  const u32 r_gp = header.initial_gp;
  const u32 r_sp = header.initial_sp_base + header.initial_sp_offset;
  const u32 r_fp = header.initial_sp_base + header.initial_sp_offset;
  return BIOS::PatchBIOSForEXE(Bus::g_bios, Bus::BIOS_SIZE, r_pc, r_gp, r_sp, r_fp);
}

bool System::LoadEXE(const char* filename)
{
  const std::string libps_path(Path::BuildRelativePath(filename, "libps.exe"));
  if (!libps_path.empty() && FileSystem::FileExists(libps_path.c_str()) && !LoadEXEToRAM(libps_path.c_str(), false))
  {
    Log_ErrorPrintf("Failed to load libps.exe from '%s'", libps_path.c_str());
    return false;
  }

  return LoadEXEToRAM(filename, true);
}

bool System::InjectEXEFromBuffer(const void* buffer, u32 buffer_size, bool patch_bios)
{
  const u8* buffer_ptr = static_cast<const u8*>(buffer);
  const u8* buffer_end = static_cast<const u8*>(buffer) + buffer_size;

  BIOS::PSEXEHeader header;
  if (buffer_size < sizeof(header))
    return false;

  std::memcpy(&header, buffer_ptr, sizeof(header));
  buffer_ptr += sizeof(header);

  const u32 file_size = static_cast<u32>(static_cast<u32>(buffer_end - buffer_ptr));
  if (!BIOS::IsValidPSExeHeader(header, file_size))
    return false;

  if (header.memfill_size > 0)
  {
    const u32 words_to_write = header.memfill_size / 4;
    u32 address = header.memfill_start & ~UINT32_C(3);
    for (u32 i = 0; i < words_to_write; i++)
    {
      CPU::SafeWriteMemoryWord(address, 0);
      address += sizeof(u32);
    }
  }

  const u32 file_data_size = std::min<u32>(file_size - sizeof(BIOS::PSEXEHeader), header.file_size);
  if (file_data_size >= 4)
  {
    std::vector<u32> data_words((file_data_size + 3) / 4);
    if ((buffer_end - buffer_ptr) < file_data_size)
      return false;

    std::memcpy(data_words.data(), buffer_ptr, file_data_size);

    const u32 num_words = file_data_size / 4;
    u32 address = header.load_address;
    for (u32 i = 0; i < num_words; i++)
    {
      CPU::SafeWriteMemoryWord(address, data_words[i]);
      address += sizeof(u32);
    }
  }

  // patch the BIOS to jump to the executable directly
  if (patch_bios)
  {
    const u32 r_pc = header.initial_pc;
    const u32 r_gp = header.initial_gp;
    const u32 r_sp = header.initial_sp_base + header.initial_sp_offset;
    const u32 r_fp = header.initial_sp_base + header.initial_sp_offset;
    if (!BIOS::PatchBIOSForEXE(Bus::g_bios, Bus::BIOS_SIZE, r_pc, r_gp, r_sp, r_fp))
      return false;
  }

  return true;
}

#if 0
// currently not used until EXP1 is implemented

bool SetExpansionROM(const char* filename)
{
  std::FILE* fp = FileSystem::OpenCFile(filename, "rb");
  if (!fp)
  {
    Log_ErrorPrintf("Failed to open '%s'", filename);
    return false;
  }

  std::fseek(fp, 0, SEEK_END);
  const u32 size = static_cast<u32>(std::ftell(fp));
  std::fseek(fp, 0, SEEK_SET);

  std::vector<u8> data(size);
  if (std::fread(data.data(), size, 1, fp) != 1)
  {
    Log_ErrorPrintf("Failed to read ROM data from '%s'", filename);
    std::fclose(fp);
    return false;
  }

  std::fclose(fp);

  Log_InfoPrintf("Loaded expansion ROM from '%s': %u bytes", filename, size);
  Bus::SetExpansionROM(std::move(data));
  return true;
}

void System::StallCPU(TickCount ticks)
{
  CPU::AddPendingTicks(ticks);
}
#endif

Controller* System::GetController(u32 slot)
{
  return Pad::GetController(slot);
}

void System::UpdateControllers()
{
  auto lock = Host::GetSettingsLock();

  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Pad::SetController(i, nullptr);

    const ControllerType type = g_settings.controller_types[i];
    if (type != ControllerType::None)
    {
      std::unique_ptr<Controller> controller = Controller::Create(type, i);
      if (controller)
      {
        controller->LoadSettings(*Host::GetSettingsInterfaceForBindings(), Controller::GetSettingsSection(i).c_str());
        Pad::SetController(i, std::move(controller));
      }
    }
  }
}

void System::UpdateControllerSettings()
{
  auto lock = Host::GetSettingsLock();

  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Controller* controller = Pad::GetController(i);
    if (controller)
      controller->LoadSettings(*Host::GetSettingsInterfaceForBindings(), Controller::GetSettingsSection(i).c_str());
  }
}

void System::ResetControllers()
{
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Controller* controller = Pad::GetController(i);
    if (controller)
      controller->Reset();
  }
}

std::unique_ptr<MemoryCard> System::GetMemoryCardForSlot(u32 slot, MemoryCardType type)
{
  // Disable memory cards when running PSFs.
  const bool is_running_psf = !s_running_game_path.empty() && IsPsfFileName(s_running_game_path.c_str());
  if (is_running_psf)
    return nullptr;

  switch (type)
  {
    case MemoryCardType::PerGame:
    {
      if (s_running_game_serial.empty())
      {
        Host::AddFormattedOSDMessage(
          5.0f,
          Host::TranslateString("System", "Per-game memory card cannot be used for slot %u as the running "
                                          "game has no code. Using shared card instead."),
          slot + 1u);
        return MemoryCard::Open(g_settings.GetSharedMemoryCardPath(slot));
      }
      else
      {
        return MemoryCard::Open(g_settings.GetGameMemoryCardPath(s_running_game_serial.c_str(), slot));
      }
    }

    case MemoryCardType::PerGameTitle:
    {
      if (s_running_game_title.empty())
      {
        Host::AddFormattedOSDMessage(
          5.0f,
          Host::TranslateString("System", "Per-game memory card cannot be used for slot %u as the running "
                                          "game has no title. Using shared card instead."),
          slot + 1u);
        return MemoryCard::Open(g_settings.GetSharedMemoryCardPath(slot));
      }
      else
      {
        return MemoryCard::Open(g_settings.GetGameMemoryCardPath(
          MemoryCard::SanitizeGameTitleForFileName(s_running_game_title).c_str(), slot));
      }
    }

    case MemoryCardType::PerGameFileTitle:
    {
      const std::string display_name(FileSystem::GetDisplayNameFromPath(s_running_game_path));
      const std::string_view file_title(Path::GetFileTitle(display_name));
      if (file_title.empty())
      {
        Host::AddFormattedOSDMessage(
          5.0f,
          Host::TranslateString("System", "Per-game memory card cannot be used for slot %u as the running "
                                          "game has no path. Using shared card instead."),
          slot + 1u);
        return MemoryCard::Open(g_settings.GetSharedMemoryCardPath(slot));
      }
      else
      {
        return MemoryCard::Open(
          g_settings.GetGameMemoryCardPath(MemoryCard::SanitizeGameTitleForFileName(file_title).c_str(), slot));
      }
    }

    case MemoryCardType::Shared:
    {
      return MemoryCard::Open(g_settings.GetSharedMemoryCardPath(slot));
    }

    case MemoryCardType::NonPersistent:
      return MemoryCard::Create();

    case MemoryCardType::None:
    default:
      return nullptr;
  }
}

void System::UpdateMemoryCardTypes()
{
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Pad::SetMemoryCard(i, nullptr);

    const MemoryCardType type = g_settings.memory_card_types[i];
    std::unique_ptr<MemoryCard> card = GetMemoryCardForSlot(i, type);
    if (card)
      Pad::SetMemoryCard(i, std::move(card));
  }
}

void System::UpdatePerGameMemoryCards()
{
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    const MemoryCardType type = g_settings.memory_card_types[i];
    if (!Settings::IsPerGameMemoryCardType(type))
      continue;

    Pad::SetMemoryCard(i, nullptr);

    std::unique_ptr<MemoryCard> card = GetMemoryCardForSlot(i, type);
    if (card)
      Pad::SetMemoryCard(i, std::move(card));
  }
}

bool System::HasMemoryCard(u32 slot)
{
  return (Pad::GetMemoryCard(slot) != nullptr);
}

void System::SwapMemoryCards()
{
  if (!IsValid())
    return;

  std::unique_ptr<MemoryCard> first = Pad::RemoveMemoryCard(0);
  std::unique_ptr<MemoryCard> second = Pad::RemoveMemoryCard(1);
  Pad::SetMemoryCard(0, std::move(second));
  Pad::SetMemoryCard(1, std::move(first));

  if (HasMemoryCard(0) && HasMemoryCard(1))
  {
    Host::AddOSDMessage(
      Host::TranslateStdString("OSDMessage", "Swapped memory card ports. Both ports have a memory card."), 10.0f);
  }
  else if (HasMemoryCard(1))
  {
    Host::AddOSDMessage(
      Host::TranslateStdString("OSDMessage", "Swapped memory card ports. Port 2 has a memory card, Port 1 is empty."),
      10.0f);
  }
  else if (HasMemoryCard(0))
  {
    Host::AddOSDMessage(
      Host::TranslateStdString("OSDMessage", "Swapped memory card ports. Port 1 has a memory card, Port 2 is empty."),
      10.0f);
  }
  else
  {
    Host::AddOSDMessage(
      Host::TranslateStdString("OSDMessage", "Swapped memory card ports. Neither port has a memory card."), 10.0f);
  }
}

void System::UpdateMultitaps()
{
  switch (g_settings.multitap_mode)
  {
    case MultitapMode::Disabled:
    {
      Pad::GetMultitap(0)->SetEnable(false, 0);
      Pad::GetMultitap(1)->SetEnable(false, 0);
    }
    break;

    case MultitapMode::Port1Only:
    {
      Pad::GetMultitap(0)->SetEnable(true, 0);
      Pad::GetMultitap(1)->SetEnable(false, 0);
    }
    break;

    case MultitapMode::Port2Only:
    {
      Pad::GetMultitap(0)->SetEnable(false, 0);
      Pad::GetMultitap(1)->SetEnable(true, 1);
    }
    break;

    case MultitapMode::BothPorts:
    {
      Pad::GetMultitap(0)->SetEnable(true, 0);
      Pad::GetMultitap(1)->SetEnable(true, 4);
    }
    break;
  }
}

bool System::DumpRAM(const char* filename)
{
  if (!IsValid())
    return false;

  return FileSystem::WriteBinaryFile(filename, Bus::g_ram, Bus::g_ram_size);
}

bool System::DumpVRAM(const char* filename)
{
  if (!IsValid())
    return false;

  g_gpu->RestoreGraphicsAPIState();
  const bool result = g_gpu->DumpVRAMToFile(filename);
  g_gpu->ResetGraphicsAPIState();

  return result;
}

bool System::DumpSPURAM(const char* filename)
{
  if (!IsValid())
    return false;

  return FileSystem::WriteBinaryFile(filename, SPU::GetRAM().data(), SPU::RAM_SIZE);
}

bool System::HasMedia()
{
  return CDROM::HasMedia();
}

std::string System::GetMediaFileName()
{
  if (!CDROM::HasMedia())
    return {};

  return CDROM::GetMediaFileName();
}

bool System::InsertMedia(const char* path)
{
  Common::Error error;
  std::unique_ptr<CDImage> image = CDImage::Open(path, g_settings.cdrom_load_image_patches, &error);
  if (!image)
  {
    Host::AddFormattedOSDMessage(10.0f, Host::TranslateString("OSDMessage", "Failed to open disc image '%s': %s."),
                                 path, error.GetCodeAndMessage().GetCharArray());
    return false;
  }

  const DiscRegion region = GetRegionForImage(image.get());
  UpdateRunningGame(path, image.get(), false);
  CDROM::InsertMedia(std::move(image), region);
  Log_InfoPrintf("Inserted media from %s (%s, %s)", s_running_game_path.c_str(), s_running_game_serial.c_str(),
                 s_running_game_title.c_str());
  if (g_settings.cdrom_load_image_to_ram)
    CDROM::PrecacheMedia();

  Host::AddFormattedOSDMessage(10.0f, Host::TranslateString("OSDMessage", "Inserted disc '%s' (%s)."),
                               s_running_game_title.c_str(), s_running_game_serial.c_str());

  if (g_settings.HasAnyPerGameMemoryCards())
  {
    Host::AddOSDMessage(Host::TranslateStdString("System", "Game changed, reloading memory cards."), 10.0f);
    UpdatePerGameMemoryCards();
  }

  ClearMemorySaveStates();
  return true;
}

void System::RemoveMedia()
{
  CDROM::RemoveMedia(false);
  ClearMemorySaveStates();
}

void System::UpdateRunningGame(const char* path, CDImage* image, bool booting)
{
  if (!booting && s_running_game_path == path)
    return;

  s_running_game_path.clear();
  s_running_game_serial.clear();
  s_running_game_title.clear();
  s_running_game_hash = 0;
  s_running_unknown_game = true;

  if (path && std::strlen(path) > 0)
  {
    s_running_game_path = path;

    if (IsExeFileName(path) || IsPsfFileName(path))
    {
      // TODO: We could pull the title from the PSF.
      s_running_game_title = Path::GetFileTitle(path);
    }
    else if (image)
    {
      std::string id;
      GetGameDetailsFromImage(image, &id, &s_running_game_hash);

      const GameDatabase::Entry* entry = GameDatabase::GetEntryForId(id);
      if (entry)
      {
        s_running_game_serial = entry->serial;
        s_running_game_title = entry->title;
        s_running_unknown_game = false;
      }
      else
      {
        s_running_game_serial = std::move(id);
        s_running_game_title = Path::GetFileTitle(FileSystem::GetDisplayNameFromPath(path));
      }

      if (image->HasSubImages() && g_settings.memory_card_use_playlist_title)
      {
        std::string image_title(image->GetMetadata("title"));
        if (!image_title.empty())
          s_running_game_title = std::move(image_title);
      }
    }
  }

  g_texture_replacements.SetGameID(s_running_game_serial);

#ifdef WITH_CHEEVOS
  if (booting)
    Achievements::ResetChallengeMode();

  Achievements::GameChanged(s_running_game_path, image);
#endif

  UpdateGameSettingsLayer();
  ApplySettings(true);

  s_cheat_list.reset();
  if (g_settings.auto_load_cheats && !Achievements::ChallengeModeActive())
    LoadCheatListFromGameTitle();

  Host::OnGameChanged(s_running_game_path, s_running_game_serial, s_running_game_title);
}

bool System::CheckForSBIFile(CDImage* image)
{
  if (s_running_game_serial.empty() || !LibcryptGameList::IsLibcryptGameCode(s_running_game_serial) || !image ||
      image->HasNonStandardSubchannel())
  {
    return true;
  }

  Log_WarningPrintf("SBI file missing but required for %s (%s)", s_running_game_serial.c_str(),
                    s_running_game_title.c_str());

  if (Host::GetBoolSettingValue("CDROM", "AllowBootingWithoutSBIFile", false))
  {
    return Host::ConfirmMessage(
      "Confirm Unsupported Configuration",
      StringUtil::StdStringFromFormat(
        Host::TranslateString(
          "System",
          "You are attempting to run a libcrypt protected game without an SBI file:\n\n%s: %s\n\nThe game will "
          "likely not run properly.\n\nPlease check the README for instructions on how to add an SBI file.\n\nDo "
          "you wish to continue?"),
        s_running_game_serial.c_str(), s_running_game_title.c_str())
        .c_str());
  }
  else
  {
    Host::ReportErrorAsync(
      Host::TranslateString("System", "Error"),
      SmallString::FromFormat(
        Host::TranslateString(
          "System",
          "You are attempting to run a libcrypt protected game without an SBI file:\n\n%s: %s\n\nYour dump is "
          "incomplete, you must add the SBI file to run this game. \n\n"
          "The name of the SBI file must match the name of the disc image."),
        s_running_game_serial.c_str(), s_running_game_title.c_str()));
    return false;
  }
}

bool System::HasMediaSubImages()
{
  const CDImage* cdi = CDROM::GetMedia();
  return cdi ? cdi->HasSubImages() : false;
}

u32 System::GetMediaSubImageCount()
{
  const CDImage* cdi = CDROM::GetMedia();
  return cdi ? cdi->GetSubImageCount() : 0;
}

u32 System::GetMediaSubImageIndex()
{
  const CDImage* cdi = CDROM::GetMedia();
  return cdi ? cdi->GetCurrentSubImage() : 0;
}

u32 System::GetMediaSubImageIndexForTitle(const std::string_view& title)
{
  const CDImage* cdi = CDROM::GetMedia();
  if (!cdi)
    return 0;

  const u32 count = cdi->GetSubImageCount();
  for (u32 i = 0; i < count; i++)
  {
    if (title == cdi->GetSubImageMetadata(i, "title"))
      return i;
  }

  return std::numeric_limits<u32>::max();
}

std::string System::GetMediaSubImageTitle(u32 index)
{
  const CDImage* cdi = CDROM::GetMedia();
  if (!cdi)
    return {};

  return cdi->GetSubImageMetadata(index, "title");
}

bool System::SwitchMediaSubImage(u32 index)
{
  if (!CDROM::HasMedia())
    return false;

  std::unique_ptr<CDImage> image = CDROM::RemoveMedia(true);
  Assert(image);

  Common::Error error;
  if (!image->SwitchSubImage(index, &error))
  {
    Host::AddFormattedOSDMessage(10.0f,
                                 Host::TranslateString("OSDMessage", "Failed to switch to subimage %u in '%s': %s."),
                                 index + 1u, image->GetFileName().c_str(), error.GetCodeAndMessage().GetCharArray());

    const DiscRegion region = GetRegionForImage(image.get());
    CDROM::InsertMedia(std::move(image), region);
    return false;
  }

  Host::AddFormattedOSDMessage(20.0f, Host::TranslateString("OSDMessage", "Switched to sub-image %s (%u) in '%s'."),
                               image->GetSubImageMetadata(index, "title").c_str(), index + 1u,
                               image->GetMetadata("title").c_str());
  const DiscRegion region = GetRegionForImage(image.get());
  CDROM::InsertMedia(std::move(image), region);

  ClearMemorySaveStates();
  return true;
}

bool System::HasCheatList()
{
  return static_cast<bool>(s_cheat_list);
}

CheatList* System::GetCheatList()
{
  return s_cheat_list.get();
}

void System::ApplyCheatCode(const CheatCode& code)
{
  Assert(!IsShutdown());
  code.Apply();
}

void System::SetCheatList(std::unique_ptr<CheatList> cheats)
{
  Assert(!IsShutdown());
  s_cheat_list = std::move(cheats);
}

void System::CheckForSettingsChanges(const Settings& old_settings)
{
  if (IsValid() && (g_settings.gpu_renderer != old_settings.gpu_renderer ||
                    g_settings.gpu_use_debug_device != old_settings.gpu_use_debug_device ||
                    g_settings.gpu_threaded_presentation != old_settings.gpu_threaded_presentation))
  {
    // if debug device/threaded presentation change, we need to recreate the whole display
    const bool recreate_display = (g_settings.gpu_use_debug_device != old_settings.gpu_use_debug_device ||
                                   g_settings.gpu_threaded_presentation != old_settings.gpu_threaded_presentation);

    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Switching to %s%s GPU renderer."),
                                 Settings::GetRendererName(g_settings.gpu_renderer),
                                 g_settings.gpu_use_debug_device ? " (debug)" : "");
    RecreateGPU(g_settings.gpu_renderer, recreate_display);
  }

  if (IsValid())
  {
    ClearMemorySaveStates();

    if (g_settings.cpu_overclock_active != old_settings.cpu_overclock_active ||
        (g_settings.cpu_overclock_active &&
         (g_settings.cpu_overclock_numerator != old_settings.cpu_overclock_numerator ||
          g_settings.cpu_overclock_denominator != old_settings.cpu_overclock_denominator)))
    {
      UpdateOverclock();
    }

    if (g_settings.audio_backend != old_settings.audio_backend ||
        g_settings.audio_driver != old_settings.audio_driver ||
        g_settings.audio_output_device != old_settings.audio_output_device)
    {
      if (g_settings.audio_backend != old_settings.audio_backend)
      {
        Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Switching to %s audio backend."),
                                     Settings::GetAudioBackendName(g_settings.audio_backend));
      }

      SPU::RecreateOutputStream();
    }
    if (g_settings.audio_stretch_mode != old_settings.audio_stretch_mode)
      SPU::GetOutputStream()->SetStretchMode(g_settings.audio_stretch_mode);
    if (g_settings.audio_buffer_ms != old_settings.audio_buffer_ms ||
        g_settings.audio_output_latency_ms != old_settings.audio_output_latency_ms ||
        g_settings.audio_stretch_mode != old_settings.audio_stretch_mode)
    {
      SPU::RecreateOutputStream();
      UpdateSpeedLimiterState();
    }

    if (g_settings.emulation_speed != old_settings.emulation_speed)
      UpdateThrottlePeriod();

    if (g_settings.cpu_execution_mode != old_settings.cpu_execution_mode ||
        g_settings.cpu_fastmem_mode != old_settings.cpu_fastmem_mode)
    {
      Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Switching to %s CPU execution mode."),
                                   Host::TranslateString("CPUExecutionMode", Settings::GetCPUExecutionModeDisplayName(
                                                                               g_settings.cpu_execution_mode))
                                     .GetCharArray());
      CPU::CodeCache::Reinitialize();
      CPU::ClearICache();
    }

    if (g_settings.cpu_execution_mode == CPUExecutionMode::Recompiler &&
        (g_settings.cpu_recompiler_memory_exceptions != old_settings.cpu_recompiler_memory_exceptions ||
         g_settings.cpu_recompiler_block_linking != old_settings.cpu_recompiler_block_linking ||
         g_settings.cpu_recompiler_icache != old_settings.cpu_recompiler_icache))
    {
      Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Recompiler options changed, flushing all blocks."),
                          5.0f);

      // changing memory exceptions can re-enable fastmem
      if (g_settings.cpu_recompiler_memory_exceptions != old_settings.cpu_recompiler_memory_exceptions)
        CPU::CodeCache::Reinitialize();
      else
        CPU::CodeCache::Flush();

      if (g_settings.cpu_recompiler_icache != old_settings.cpu_recompiler_icache)
        CPU::ClearICache();
    }

    SPU::GetOutputStream()->SetOutputVolume(GetAudioOutputVolume());

    if (g_settings.gpu_resolution_scale != old_settings.gpu_resolution_scale ||
        g_settings.gpu_multisamples != old_settings.gpu_multisamples ||
        g_settings.gpu_per_sample_shading != old_settings.gpu_per_sample_shading ||
        g_settings.gpu_use_thread != old_settings.gpu_use_thread ||
        g_settings.gpu_use_software_renderer_for_readbacks != old_settings.gpu_use_software_renderer_for_readbacks ||
        g_settings.gpu_fifo_size != old_settings.gpu_fifo_size ||
        g_settings.gpu_max_run_ahead != old_settings.gpu_max_run_ahead ||
        g_settings.gpu_true_color != old_settings.gpu_true_color ||
        g_settings.gpu_scaled_dithering != old_settings.gpu_scaled_dithering ||
        g_settings.gpu_texture_filter != old_settings.gpu_texture_filter ||
        g_settings.gpu_disable_interlacing != old_settings.gpu_disable_interlacing ||
        g_settings.gpu_force_ntsc_timings != old_settings.gpu_force_ntsc_timings ||
        g_settings.gpu_24bit_chroma_smoothing != old_settings.gpu_24bit_chroma_smoothing ||
        g_settings.gpu_downsample_mode != old_settings.gpu_downsample_mode ||
        g_settings.display_crop_mode != old_settings.display_crop_mode ||
        g_settings.display_aspect_ratio != old_settings.display_aspect_ratio ||
        g_settings.display_alignment != old_settings.display_alignment ||
        g_settings.display_show_gpu != old_settings.display_show_gpu ||
        g_settings.gpu_pgxp_enable != old_settings.gpu_pgxp_enable ||
        g_settings.gpu_pgxp_texture_correction != old_settings.gpu_pgxp_texture_correction ||
        g_settings.gpu_pgxp_color_correction != old_settings.gpu_pgxp_color_correction ||
        g_settings.gpu_pgxp_depth_buffer != old_settings.gpu_pgxp_depth_buffer ||
        g_settings.display_active_start_offset != old_settings.display_active_start_offset ||
        g_settings.display_active_end_offset != old_settings.display_active_end_offset ||
        g_settings.display_line_start_offset != old_settings.display_line_start_offset ||
        g_settings.display_line_end_offset != old_settings.display_line_end_offset ||
        g_settings.rewind_enable != old_settings.rewind_enable ||
        g_settings.runahead_frames != old_settings.runahead_frames)
    {
      g_gpu->UpdateSettings();
      Host::InvalidateDisplay();
    }

    if (g_settings.gpu_widescreen_hack != old_settings.gpu_widescreen_hack ||
        g_settings.display_aspect_ratio != old_settings.display_aspect_ratio ||
        (g_settings.display_aspect_ratio == DisplayAspectRatio::Custom &&
         (g_settings.display_aspect_ratio_custom_numerator != old_settings.display_aspect_ratio_custom_numerator ||
          g_settings.display_aspect_ratio_custom_denominator != old_settings.display_aspect_ratio_custom_denominator)))
    {
      GTE::UpdateAspectRatio();
    }

    if (g_settings.gpu_pgxp_enable != old_settings.gpu_pgxp_enable ||
        (g_settings.gpu_pgxp_enable && (g_settings.gpu_pgxp_culling != old_settings.gpu_pgxp_culling ||
                                        g_settings.gpu_pgxp_vertex_cache != old_settings.gpu_pgxp_vertex_cache ||
                                        g_settings.gpu_pgxp_cpu != old_settings.gpu_pgxp_cpu)))
    {
      if (g_settings.IsUsingCodeCache())
      {
        Host::AddOSDMessage(g_settings.gpu_pgxp_enable ?
                              Host::TranslateStdString("OSDMessage", "PGXP enabled, recompiling all blocks.") :
                              Host::TranslateStdString("OSDMessage", "PGXP disabled, recompiling all blocks."),
                            5.0f);
        CPU::CodeCache::Flush();
      }

      if (old_settings.gpu_pgxp_enable)
        PGXP::Shutdown();

      if (g_settings.gpu_pgxp_enable)
        PGXP::Initialize();
    }

    if (g_settings.cdrom_readahead_sectors != old_settings.cdrom_readahead_sectors)
      CDROM::SetReadaheadSectors(g_settings.cdrom_readahead_sectors);

    if (g_settings.memory_card_types != old_settings.memory_card_types ||
        g_settings.memory_card_paths != old_settings.memory_card_paths ||
        (g_settings.memory_card_use_playlist_title != old_settings.memory_card_use_playlist_title &&
         HasMediaSubImages()))
    {
      UpdateMemoryCardTypes();
    }

    if (g_settings.rewind_enable != old_settings.rewind_enable ||
        g_settings.rewind_save_frequency != old_settings.rewind_save_frequency ||
        g_settings.rewind_save_slots != old_settings.rewind_save_slots ||
        g_settings.runahead_frames != old_settings.runahead_frames)
    {
      UpdateMemorySaveStateSettings();
    }

    if (g_settings.texture_replacements.enable_vram_write_replacements !=
          old_settings.texture_replacements.enable_vram_write_replacements ||
        g_settings.texture_replacements.preload_textures != old_settings.texture_replacements.preload_textures)
    {
      g_texture_replacements.Reload();
    }

    DMA::SetMaxSliceTicks(g_settings.dma_max_slice_ticks);
    DMA::SetHaltTicks(g_settings.dma_halt_ticks);

    if (g_settings.audio_backend != old_settings.audio_backend ||
        g_settings.video_sync_enabled != old_settings.video_sync_enabled ||
        g_settings.increase_timer_resolution != old_settings.increase_timer_resolution ||
        g_settings.emulation_speed != old_settings.emulation_speed ||
        g_settings.fast_forward_speed != old_settings.fast_forward_speed ||
        g_settings.display_max_fps != old_settings.display_max_fps ||
        g_settings.display_all_frames != old_settings.display_all_frames ||
        g_settings.sync_to_host_refresh_rate != old_settings.sync_to_host_refresh_rate)
    {
      UpdateSpeedLimiterState();
    }

    if (g_settings.display_post_processing != old_settings.display_post_processing ||
        g_settings.display_post_process_chain != old_settings.display_post_process_chain)
    {
      if (g_settings.display_post_processing && !g_settings.display_post_process_chain.empty())
      {
        if (!g_host_display->SetPostProcessingChain(g_settings.display_post_process_chain))
          Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Failed to load post processing shader chain."),
                              20.0f);
      }
      else
      {
        g_host_display->SetPostProcessingChain({});
      }
    }
  }

  bool controllers_updated = false;
  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    if (g_settings.controller_types[i] != old_settings.controller_types[i])
    {
      if (IsValid() && !controllers_updated)
      {
        UpdateControllers();
        ResetControllers();
        UpdateSoftwareCursor();
        controllers_updated = true;
      }
    }

    if (IsValid() && !controllers_updated)
    {
      UpdateControllerSettings();
      UpdateSoftwareCursor();
    }
  }

  if (g_settings.multitap_mode != old_settings.multitap_mode)
    UpdateMultitaps();
}

void System::CalculateRewindMemoryUsage(u32 num_saves, u64* ram_usage, u64* vram_usage)
{
  *ram_usage = MAX_SAVE_STATE_SIZE * static_cast<u64>(num_saves);
  *vram_usage = (VRAM_WIDTH * VRAM_HEIGHT * 4) * static_cast<u64>(std::max(g_settings.gpu_resolution_scale, 1u)) *
                static_cast<u64>(g_settings.gpu_multisamples) * static_cast<u64>(num_saves);
}

void System::ClearMemorySaveStates()
{
  s_rewind_states.clear();
  s_runahead_states.clear();
}

void System::UpdateMemorySaveStateSettings()
{
  ClearMemorySaveStates();

  s_memory_saves_enabled = g_settings.rewind_enable;

  if (g_settings.rewind_enable)
  {
    s_rewind_save_frequency = static_cast<s32>(std::ceil(g_settings.rewind_save_frequency * s_throttle_frequency));
    s_rewind_save_counter = 0;

    u64 ram_usage, vram_usage;
    CalculateRewindMemoryUsage(g_settings.rewind_save_slots, &ram_usage, &vram_usage);
    Log_InfoPrintf(
      "Rewind is enabled, saving every %d frames, with %u slots and %" PRIu64 "MB RAM and %" PRIu64 "MB VRAM usage",
      std::max(s_rewind_save_frequency, 1), g_settings.rewind_save_slots, ram_usage / 1048576, vram_usage / 1048576);
  }
  else
  {
    s_rewind_save_frequency = -1;
    s_rewind_save_counter = -1;
  }

  s_rewind_load_frequency = -1;
  s_rewind_load_counter = -1;

  s_runahead_frames = g_settings.runahead_frames;
  s_runahead_replay_pending = false;
  if (s_runahead_frames > 0)
    Log_InfoPrintf("Runahead is active with %u frames", s_runahead_frames);
}

bool System::LoadMemoryState(const MemorySaveState& mss)
{
  mss.state_stream->SeekAbsolute(0);

  StateWrapper sw(mss.state_stream.get(), StateWrapper::Mode::Read, SAVE_STATE_VERSION);
  GPUTexture* host_texture = mss.vram_texture.get();
  if (!DoState(sw, &host_texture, true, true))
  {
    Host::ReportErrorAsync("Error", "Failed to load memory save state, resetting.");
    InternalReset();
    return false;
  }

  return true;
}

bool System::SaveMemoryState(MemorySaveState* mss)
{
  if (!mss->state_stream)
    mss->state_stream = std::make_unique<GrowableMemoryByteStream>(nullptr, MAX_SAVE_STATE_SIZE);
  else
    mss->state_stream->SeekAbsolute(0);

  GPUTexture* host_texture = mss->vram_texture.release();
  StateWrapper sw(mss->state_stream.get(), StateWrapper::Mode::Write, SAVE_STATE_VERSION);
  if (!DoState(sw, &host_texture, false, true))
  {
    Log_ErrorPrint("Failed to create rewind state.");
    delete host_texture;
    return false;
  }

  mss->vram_texture.reset(host_texture);
  return true;
}

bool System::SaveRewindState()
{
#ifdef PROFILE_MEMORY_SAVE_STATES
  Common::Timer save_timer;
#endif

  // try to reuse the frontmost slot
  const u32 save_slots = g_settings.rewind_save_slots;
  MemorySaveState mss;
  while (s_rewind_states.size() >= save_slots)
  {
    mss = std::move(s_rewind_states.front());
    s_rewind_states.pop_front();
  }

  if (!SaveMemoryState(&mss))
    return false;

  s_rewind_states.push_back(std::move(mss));

#ifdef PROFILE_MEMORY_SAVE_STATES
  Log_DevPrintf("Saved rewind state (%" PRIu64 " bytes, took %.4f ms)", s_rewind_states.back().state_stream->GetSize(),
                save_timer.GetTimeMilliseconds());
#endif

  return true;
}

bool System::LoadRewindState(u32 skip_saves /*= 0*/, bool consume_state /*=true */)
{
  while (skip_saves > 0 && !s_rewind_states.empty())
  {
    s_rewind_states.pop_back();
    skip_saves--;
  }

  if (s_rewind_states.empty())
    return false;

#ifdef PROFILE_MEMORY_SAVE_STATES
  Common::Timer load_timer;
#endif

  if (!LoadMemoryState(s_rewind_states.back()))
    return false;

  if (consume_state)
    s_rewind_states.pop_back();

#ifdef PROFILE_MEMORY_SAVE_STATES
  Log_DevPrintf("Rewind load took %.4f ms", load_timer.GetTimeMilliseconds());
#endif

  return true;
}

bool System::IsRewinding()
{
  return (s_rewind_load_frequency >= 0);
}

void System::SetRewinding(bool enabled)
{
  if (enabled)
  {
    // Try to rewind at the replay speed, or one per second maximum.
    const float load_frequency = std::min(g_settings.rewind_save_frequency, 1.0f);
    s_rewind_load_frequency = static_cast<s32>(std::ceil(load_frequency * s_throttle_frequency));
    s_rewind_load_counter = 0;
  }
  else
  {
    s_rewind_load_frequency = -1;
    s_rewind_load_counter = -1;
  }

  s_rewinding_first_save = true;
}

void System::DoRewind()
{
  if (s_rewind_load_counter == 0)
  {
    const u32 skip_saves = BoolToUInt32(!s_rewinding_first_save);
    s_rewinding_first_save = false;
    LoadRewindState(skip_saves, false);
    ResetPerformanceCounters();
    s_rewind_load_counter = s_rewind_load_frequency;
  }
  else
  {
    s_rewind_load_counter--;
  }

  s_next_frame_time += s_frame_period;
}

void System::SaveRunaheadState()
{
  // try to reuse the frontmost slot
  MemorySaveState mss;
  while (s_runahead_states.size() >= s_runahead_frames)
  {
    mss = std::move(s_runahead_states.front());
    s_runahead_states.pop_front();
  }

  if (!SaveMemoryState(&mss))
  {
    Log_ErrorPrint("Failed to save runahead state.");
    return;
  }

  s_runahead_states.push_back(std::move(mss));
}

void System::DoRunahead()
{
#ifdef PROFILE_MEMORY_SAVE_STATES
  Common::Timer timer;
  Log_DevPrintf("runahead starting at frame %u", s_frame_number);
#endif

  if (s_runahead_replay_pending)
  {
    // we need to replay and catch up - load the state,
    s_runahead_replay_pending = false;
    if (s_runahead_states.empty() || !LoadMemoryState(s_runahead_states.front()))
    {
      s_runahead_states.clear();
      return;
    }

    // and throw away all the states, forcing us to catch up below
    // TODO: can we leave one frame here and run, avoiding the extra save?
    s_runahead_states.clear();

#ifdef PROFILE_MEMORY_SAVE_STATES
    Log_VerbosePrintf("Rewound to frame %u, took %.2f ms", s_frame_number, timer.GetTimeMilliseconds());
#endif
  }

  // run the frames with no audio
  s32 frames_to_run = static_cast<s32>(s_runahead_frames) - static_cast<s32>(s_runahead_states.size());
  if (frames_to_run > 0)
  {
    Common::Timer timer2;
#ifdef PROFILE_MEMORY_SAVE_STATES
    const s32 temp = frames_to_run;
#endif

    SPU::SetAudioOutputMuted(true);

    while (frames_to_run > 0)
    {
      DoRunFrame();
      SaveRunaheadState();
      frames_to_run--;
    }

    SPU::SetAudioOutputMuted(false);

#ifdef PROFILE_MEMORY_SAVE_STATES
    Log_VerbosePrintf("Running %d frames to catch up took %.2f ms", temp, timer2.GetTimeMilliseconds());
#endif
  }
  else
  {
    // save this frame
    SaveRunaheadState();
  }

#ifdef PROFILE_MEMORY_SAVE_STATES
  Log_DevPrintf("runahead ending at frame %u, took %.2f ms", s_frame_number, timer.GetTimeMilliseconds());
#endif
}

void System::DoMemorySaveStates()
{
  if (s_rewind_save_counter >= 0)
  {
    if (s_rewind_save_counter == 0)
    {
      SaveRewindState();
      s_rewind_save_counter = s_rewind_save_frequency;
    }
    else
    {
      s_rewind_save_counter--;
    }
  }

  if (s_runahead_frames > 0)
    SaveRunaheadState();
}

void System::SetRunaheadReplayFlag()
{
  if (s_runahead_frames == 0 || s_runahead_states.empty())
    return;

#ifdef PROFILE_MEMORY_SAVE_STATES
  Log_DevPrintf("Runahead rewind pending...");
#endif

  s_runahead_replay_pending = true;
}

void System::ShutdownSystem(bool save_resume_state)
{
  if (!IsValid())
    return;

  if (save_resume_state)
    SaveResumeState();

  DestroySystem();
}

bool System::CanUndoLoadState()
{
  return static_cast<bool>(m_undo_load_state);
}

std::optional<ExtendedSaveStateInfo> System::GetUndoSaveStateInfo()
{
  std::optional<ExtendedSaveStateInfo> ssi;
  if (m_undo_load_state)
  {
    m_undo_load_state->SeekAbsolute(0);
    ssi = InternalGetExtendedSaveStateInfo(m_undo_load_state.get());
    m_undo_load_state->SeekAbsolute(0);

    if (ssi)
      ssi->timestamp = 0;
  }

  return ssi;
}

bool System::UndoLoadState()
{
  if (!m_undo_load_state)
    return false;

  Assert(IsValid());

  m_undo_load_state->SeekAbsolute(0);
  if (!DoLoadState(m_undo_load_state.get(), false, true))
  {
    Host::ReportErrorAsync("Error", "Failed to load undo state, resetting system.");
    m_undo_load_state.reset();
    ResetSystem();
    return false;
  }

  Log_InfoPrintf("Loaded undo save state.");
  m_undo_load_state.reset();
  return true;
}

bool System::SaveUndoLoadState()
{
  if (m_undo_load_state)
    m_undo_load_state.reset();

  m_undo_load_state = ByteStream::CreateGrowableMemoryStream(nullptr, System::MAX_SAVE_STATE_SIZE);
  if (!InternalSaveState(m_undo_load_state.get()))
  {
    Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Failed to save undo load state."), 15.0f);
    m_undo_load_state.reset();
    return false;
  }

  Log_InfoPrintf("Saved undo load state: %" PRIu64 " bytes", m_undo_load_state->GetSize());
  return true;
}

bool System::IsRunningAtNonStandardSpeed()
{
  if (!IsValid())
    return false;

  const float target_speed = System::GetTargetSpeed();
  return (target_speed <= 0.95f || target_speed >= 1.05f);
}

s32 System::GetAudioOutputVolume()
{
  return g_settings.GetAudioOutputVolume(IsRunningAtNonStandardSpeed());
}

void System::UpdateVolume()
{
  if (!IsValid())
    return;

  SPU::GetOutputStream()->SetOutputVolume(GetAudioOutputVolume());
}

bool System::IsDumpingAudio()
{
  return SPU::IsDumpingAudio();
}

bool System::StartDumpingAudio(const char* filename)
{
  if (System::IsShutdown())
    return false;

  std::string auto_filename;
  if (!filename)
  {
    const auto& serial = System::GetGameSerial();
    if (serial.empty())
    {
      auto_filename = Path::Combine(
        EmuFolders::Dumps, fmt::format("audio" FS_OSPATH_SEPARATOR_STR "{}.wav", GetTimestampStringForFileName()));
    }
    else
    {
      auto_filename = Path::Combine(EmuFolders::Dumps, fmt::format("audio" FS_OSPATH_SEPARATOR_STR "{}_{}.wav", serial,
                                                                   GetTimestampStringForFileName()));
    }

    filename = auto_filename.c_str();
  }

  if (SPU::StartDumpingAudio(filename))
  {
    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Started dumping audio to '%s'."), filename);
    return true;
  }
  else
  {
    Host::AddFormattedOSDMessage(10.0f, Host::TranslateString("OSDMessage", "Failed to start dumping audio to '%s'."),
                                 filename);
    return false;
  }
}

void System::StopDumpingAudio()
{
  if (System::IsShutdown() || !SPU::StopDumpingAudio())
    return;

  Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Stopped dumping audio."), 5.0f);
}

bool System::SaveScreenshot(const char* filename /* = nullptr */, bool full_resolution /* = true */,
                            bool apply_aspect_ratio /* = true */, bool compress_on_thread /* = true */)
{
  if (!System::IsValid())
    return false;

  std::string auto_filename;
  if (!filename)
  {
    const auto& code = System::GetGameSerial();
    const char* extension = "png";
    if (code.empty())
    {
      auto_filename =
        Path::Combine(EmuFolders::Screenshots, fmt::format("{}.{}", GetTimestampStringForFileName(), extension));
    }
    else
    {
      auto_filename = Path::Combine(EmuFolders::Screenshots,
                                    fmt::format("{}_{}.{}", code, GetTimestampStringForFileName(), extension));
    }

    filename = auto_filename.c_str();
  }

  if (FileSystem::FileExists(filename))
  {
    Host::AddFormattedOSDMessage(10.0f, Host::TranslateString("OSDMessage", "Screenshot file '%s' already exists."),
                                 filename);
    return false;
  }

  const bool screenshot_saved = g_host_display->WriteScreenshotToFile(
    filename, g_settings.display_internal_resolution_screenshots, compress_on_thread);

  if (!screenshot_saved)
  {
    Host::AddFormattedOSDMessage(10.0f, Host::TranslateString("OSDMessage", "Failed to save screenshot to '%s'"),
                                 filename);
    return false;
  }

  Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Screenshot saved to '%s'."), filename);
  return true;
}

std::string System::GetGameSaveStateFileName(const std::string_view& serial, s32 slot)
{
  if (slot < 0)
    return Path::Combine(EmuFolders::SaveStates, fmt::format("{}_resume.sav", serial));
  else
    return Path::Combine(EmuFolders::SaveStates, fmt::format("{}_{}.sav", serial, slot));
}

std::string System::GetGlobalSaveStateFileName(s32 slot)
{
  if (slot < 0)
    return Path::Combine(EmuFolders::SaveStates, "resume.sav");
  else
    return Path::Combine(EmuFolders::SaveStates, fmt::format("savestate_{}.sav", slot));
}

std::vector<SaveStateInfo> System::GetAvailableSaveStates(const char* serial)
{
  std::vector<SaveStateInfo> si;
  std::string path;

  auto add_path = [&si](std::string path, s32 slot, bool global) {
    FILESYSTEM_STAT_DATA sd;
    if (!FileSystem::StatFile(path.c_str(), &sd))
      return;

    si.push_back(SaveStateInfo{std::move(path), sd.ModificationTime, static_cast<s32>(slot), global});
  };

  if (serial && std::strlen(serial) > 0)
  {
    add_path(GetGameSaveStateFileName(serial, -1), -1, false);
    for (s32 i = 1; i <= PER_GAME_SAVE_STATE_SLOTS; i++)
      add_path(GetGameSaveStateFileName(serial, i), i, false);
  }

  for (s32 i = 1; i <= GLOBAL_SAVE_STATE_SLOTS; i++)
    add_path(GetGlobalSaveStateFileName(i), i, true);

  return si;
}

std::optional<SaveStateInfo> System::GetSaveStateInfo(const char* serial, s32 slot)
{
  const bool global = (!serial || serial[0] == 0);
  std::string path = global ? GetGlobalSaveStateFileName(slot) : GetGameSaveStateFileName(serial, slot);

  FILESYSTEM_STAT_DATA sd;
  if (!FileSystem::StatFile(path.c_str(), &sd))
    return std::nullopt;

  return SaveStateInfo{std::move(path), sd.ModificationTime, slot, global};
}

std::optional<ExtendedSaveStateInfo> System::GetExtendedSaveStateInfo(const char* path)
{
  FILESYSTEM_STAT_DATA sd;
  if (!FileSystem::StatFile(path, &sd))
    return std::nullopt;

  std::unique_ptr<ByteStream> stream = ByteStream::OpenFile(path, BYTESTREAM_OPEN_READ | BYTESTREAM_OPEN_SEEKABLE);
  if (!stream)
    return std::nullopt;

  std::optional<ExtendedSaveStateInfo> ssi(InternalGetExtendedSaveStateInfo(stream.get()));
  if (ssi)
    ssi->timestamp = sd.ModificationTime;

  return ssi;
}

std::optional<ExtendedSaveStateInfo> System::InternalGetExtendedSaveStateInfo(ByteStream* stream)
{
  SAVE_STATE_HEADER header;
  if (!stream->Read(&header, sizeof(header)) || header.magic != SAVE_STATE_MAGIC)
    return std::nullopt;

  ExtendedSaveStateInfo ssi;
  if (header.version < SAVE_STATE_MINIMUM_VERSION || header.version > SAVE_STATE_VERSION)
  {
    ssi.title = StringUtil::StdStringFromFormat(
      Host::TranslateString("CommonHostInterface", "Invalid version %u (%s version %u)"), header.version,
      header.version > SAVE_STATE_VERSION ? "maximum" : "minimum",
      header.version > SAVE_STATE_VERSION ? SAVE_STATE_VERSION : SAVE_STATE_MINIMUM_VERSION);
    return ssi;
  }

  header.title[sizeof(header.title) - 1] = 0;
  ssi.title = header.title;
  header.serial[sizeof(header.serial) - 1] = 0;
  ssi.serial = header.serial;

  if (header.media_filename_length > 0 &&
      (header.offset_to_media_filename + header.media_filename_length) <= stream->GetSize())
  {
    stream->SeekAbsolute(header.offset_to_media_filename);
    ssi.media_path.resize(header.media_filename_length);
    if (!stream->Read2(ssi.media_path.data(), header.media_filename_length))
      std::string().swap(ssi.media_path);
  }

  if (header.screenshot_width > 0 && header.screenshot_height > 0 && header.screenshot_size > 0 &&
      (static_cast<u64>(header.offset_to_screenshot) + static_cast<u64>(header.screenshot_size)) <= stream->GetSize())
  {
    stream->SeekAbsolute(header.offset_to_screenshot);
    ssi.screenshot_data.resize((header.screenshot_size + 3u) / 4u);
    if (stream->Read2(ssi.screenshot_data.data(), header.screenshot_size))
    {
      ssi.screenshot_width = header.screenshot_width;
      ssi.screenshot_height = header.screenshot_height;
    }
    else
    {
      decltype(ssi.screenshot_data)().swap(ssi.screenshot_data);
    }
  }

  return ssi;
}

void System::DeleteSaveStates(const char* serial, bool resume)
{
  const std::vector<SaveStateInfo> states(GetAvailableSaveStates(serial));
  for (const SaveStateInfo& si : states)
  {
    if (si.global || (!resume && si.slot < 0))
      continue;

    Log_InfoPrintf("Removing save state at '%s'", si.path.c_str());
    if (!FileSystem::DeleteFile(si.path.c_str()))
      Log_ErrorPrintf("Failed to delete save state file '%s'", si.path.c_str());
  }
}

std::string System::GetMostRecentResumeSaveStatePath()
{
  std::vector<FILESYSTEM_FIND_DATA> files;
  if (!FileSystem::FindFiles(EmuFolders::SaveStates.c_str(), "*resume.sav", FILESYSTEM_FIND_FILES, &files) ||
      files.empty())
  {
    return {};
  }

  FILESYSTEM_FIND_DATA* most_recent = &files[0];
  for (FILESYSTEM_FIND_DATA& file : files)
  {
    if (file.ModificationTime > most_recent->ModificationTime)
      most_recent = &file;
  }

  return std::move(most_recent->FileName);
}

std::string System::GetCheatFileName()
{
  std::string ret;

  const std::string& title = System::GetGameTitle();
  if (!title.empty())
    ret = Path::Combine(EmuFolders::Cheats, fmt::format("{}.cht", title.c_str()));

  return ret;
}

bool System::LoadCheatList(const char* filename)
{
  if (System::IsShutdown())
    return false;

  std::unique_ptr<CheatList> cl = std::make_unique<CheatList>();
  if (!cl->LoadFromFile(filename, CheatList::Format::Autodetect))
  {
    Host::AddFormattedOSDMessage(15.0f, Host::TranslateString("OSDMessage", "Failed to load cheats from '%s'."),
                                 filename);
    return false;
  }

  if (cl->GetEnabledCodeCount() > 0)
  {
    Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "%n cheats are enabled. This may result in instability.",
                                                 "", cl->GetEnabledCodeCount()),
                        30.0f);
  }

  System::SetCheatList(std::move(cl));
  return true;
}

bool System::LoadCheatListFromGameTitle()
{
  // Called when booting, needs to test for shutdown.
  if (IsShutdown() || Achievements::ChallengeModeActive())
    return false;

  const std::string filename(GetCheatFileName());
  if (filename.empty() || !FileSystem::FileExists(filename.c_str()))
    return false;

  return LoadCheatList(filename.c_str());
}

bool System::LoadCheatListFromDatabase()
{
  if (IsShutdown() || s_running_game_serial.empty() || Achievements::ChallengeModeActive())
    return false;

  std::unique_ptr<CheatList> cl = std::make_unique<CheatList>();
  if (!cl->LoadFromPackage(s_running_game_serial))
    return false;

  Log_InfoPrintf("Loaded %u cheats from database.", cl->GetCodeCount());
  SetCheatList(std::move(cl));
  return true;
}

bool System::SaveCheatList()
{
  if (!System::IsValid() || !System::HasCheatList())
    return false;

  const std::string filename(GetCheatFileName());
  if (filename.empty())
    return false;

  if (!System::GetCheatList()->SaveToPCSXRFile(filename.c_str()))
  {
    Host::AddFormattedOSDMessage(15.0f, Host::TranslateString("OSDMessage", "Failed to save cheat list to '%s'"),
                                 filename.c_str());
  }

  return true;
}

bool System::SaveCheatList(const char* filename)
{
  if (!System::IsValid() || !System::HasCheatList())
    return false;

  if (!System::GetCheatList()->SaveToPCSXRFile(filename))
    return false;

  // This shouldn't be needed, but lupdate doesn't gather this string otherwise...
  const u32 code_count = System::GetCheatList()->GetCodeCount();
  Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Saved %n cheats to '%s'.", "", code_count),
                               filename);
  return true;
}

bool System::DeleteCheatList()
{
  if (!System::IsValid())
    return false;

  const std::string filename(GetCheatFileName());
  if (!filename.empty())
  {
    if (!FileSystem::DeleteFile(filename.c_str()))
      return false;

    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Deleted cheat list '%s'."),
                                 filename.c_str());
  }

  System::SetCheatList(nullptr);
  return true;
}

void System::ClearCheatList(bool save_to_file)
{
  if (!System::IsValid())
    return;

  CheatList* cl = System::GetCheatList();
  if (!cl)
    return;

  while (cl->GetCodeCount() > 0)
    cl->RemoveCode(cl->GetCodeCount() - 1);

  if (save_to_file)
    SaveCheatList();
}

void System::SetCheatCodeState(u32 index, bool enabled, bool save_to_file)
{
  if (!System::IsValid() || !System::HasCheatList())
    return;

  CheatList* cl = System::GetCheatList();
  if (index >= cl->GetCodeCount())
    return;

  CheatCode& cc = cl->GetCode(index);
  if (cc.enabled == enabled)
    return;

  cc.enabled = enabled;
  if (!enabled)
    cc.ApplyOnDisable();

  if (enabled)
  {
    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Cheat '%s' enabled."),
                                 cc.description.c_str());
  }
  else
  {
    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Cheat '%s' disabled."),
                                 cc.description.c_str());
  }

  if (save_to_file)
    SaveCheatList();
}

void System::ApplyCheatCode(u32 index)
{
  if (!System::HasCheatList() || index >= System::GetCheatList()->GetCodeCount())
    return;

  const CheatCode& cc = System::GetCheatList()->GetCode(index);
  if (!cc.enabled)
  {
    cc.Apply();
    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Applied cheat '%s'."),
                                 cc.description.c_str());
  }
  else
  {
    Host::AddFormattedOSDMessage(5.0f, Host::TranslateString("OSDMessage", "Cheat '%s' is already enabled."),
                                 cc.description.c_str());
  }
}

void System::TogglePostProcessing()
{
  if (!IsValid())
    return;

  g_settings.display_post_processing = !g_settings.display_post_processing;
  if (g_settings.display_post_processing)
  {
    Host::AddKeyedOSDMessage("PostProcessing",
                             Host::TranslateStdString("OSDMessage", "Post-processing is now enabled."), 10.0f);

    if (!g_host_display->SetPostProcessingChain(g_settings.display_post_process_chain))
      Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Failed to load post processing shader chain."),
                          20.0f);
  }
  else
  {
    Host::AddKeyedOSDMessage("PostProcessing",
                             Host::TranslateStdString("OSDMessage", "Post-processing is now disabled."), 10.0f);
    g_host_display->SetPostProcessingChain({});
  }
}

void System::ReloadPostProcessingShaders()
{
  if (!IsValid() || !g_settings.display_post_processing)
    return;

  if (!g_host_display->SetPostProcessingChain(g_settings.display_post_process_chain))
    Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Failed to load post-processing shader chain."), 20.0f);
  else
    Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Post-processing shaders reloaded."), 10.0f);
}

void System::ToggleWidescreen()
{
  g_settings.gpu_widescreen_hack = !g_settings.gpu_widescreen_hack;

  const DisplayAspectRatio user_ratio =
    Settings::ParseDisplayAspectRatio(
      Host::GetStringSettingValue("Display", "AspectRatio",
                                  Settings::GetDisplayAspectRatioName(Settings::DEFAULT_DISPLAY_ASPECT_RATIO))
        .c_str())
      .value_or(DisplayAspectRatio::Auto);
  ;

  if (user_ratio == DisplayAspectRatio::Auto || user_ratio == DisplayAspectRatio::PAR1_1 ||
      user_ratio == DisplayAspectRatio::R4_3)
  {
    g_settings.display_aspect_ratio = g_settings.gpu_widescreen_hack ? DisplayAspectRatio::R16_9 : user_ratio;
  }
  else
  {
    g_settings.display_aspect_ratio = g_settings.gpu_widescreen_hack ? user_ratio : DisplayAspectRatio::Auto;
  }

  if (g_settings.gpu_widescreen_hack)
  {
    Host::AddKeyedFormattedOSDMessage(
      "WidescreenHack", 5.0f,
      Host::TranslateString("OSDMessage", "Widescreen hack is now enabled, and aspect ratio is set to %s."),
      Host::TranslateString("DisplayAspectRatio", Settings::GetDisplayAspectRatioName(g_settings.display_aspect_ratio))
        .GetCharArray());
  }
  else
  {
    Host::AddKeyedFormattedOSDMessage(
      "WidescreenHack", 5.0f,
      Host::TranslateString("OSDMessage", "Widescreen hack is now disabled, and aspect ratio is set to %s."),
      Host::TranslateString("DisplayAspectRatio", Settings::GetDisplayAspectRatioName(g_settings.display_aspect_ratio))
        .GetCharArray());
  }

  GTE::UpdateAspectRatio();
}

void System::ToggleSoftwareRendering()
{
  if (IsShutdown() || g_settings.gpu_renderer == GPURenderer::Software)
    return;

  const GPURenderer new_renderer = g_gpu->IsHardwareRenderer() ? GPURenderer::Software : g_settings.gpu_renderer;

  Host::AddKeyedFormattedOSDMessage("SoftwareRendering", 5.0f,
                                    Host::TranslateString("OSDMessage", "Switching to %s renderer..."),
                                    Settings::GetRendererDisplayName(new_renderer));
  RecreateGPU(new_renderer);
  Host::InvalidateDisplay();
  ResetPerformanceCounters();
}

void System::UpdateSoftwareCursor()
{
  if (!IsValid())
  {
    Host::SetMouseMode(false, false);
    g_host_display->ClearSoftwareCursor();
    return;
  }

  const Common::RGBA8Image* image = nullptr;
  float image_scale = 1.0f;
  bool relative_mode = false;
  bool hide_cursor = false;

  for (u32 i = 0; i < NUM_CONTROLLER_AND_CARD_PORTS; i++)
  {
    Controller* controller = System::GetController(i);
    if (controller && controller->GetSoftwareCursor(&image, &image_scale, &relative_mode))
    {
      hide_cursor = true;
      break;
    }
  }

  Host::SetMouseMode(relative_mode, hide_cursor);

  if (image && image->IsValid())
  {
    g_host_display->SetSoftwareCursor(image->GetPixels(), image->GetWidth(), image->GetHeight(), image->GetPitch(),
                                      image_scale);
  }
  else
  {
    g_host_display->ClearSoftwareCursor();
  }
}

void System::RequestDisplaySize(float scale /*= 0.0f*/)
{
  if (!IsValid())
    return;

  if (scale == 0.0f)
    scale = g_gpu->IsHardwareRenderer() ? static_cast<float>(g_settings.gpu_resolution_scale) : 1.0f;

  const float y_scale =
    (static_cast<float>(g_host_display->GetDisplayWidth()) / static_cast<float>(g_host_display->GetDisplayHeight())) /
    g_host_display->GetDisplayAspectRatio();

  const u32 requested_width =
    std::max<u32>(static_cast<u32>(std::ceil(static_cast<float>(g_host_display->GetDisplayWidth()) * scale)), 1);
  const u32 requested_height = std::max<u32>(
    static_cast<u32>(std::ceil(static_cast<float>(g_host_display->GetDisplayHeight()) * y_scale * scale)), 1);

  Host::RequestResizeHostDisplay(static_cast<s32>(requested_width), static_cast<s32>(requested_height));
}

void System::HostDisplayResized()
{
  if (!IsValid())
    return;

  if (g_settings.gpu_widescreen_hack && g_settings.display_aspect_ratio == DisplayAspectRatio::MatchWindow)
    GTE::UpdateAspectRatio();

  g_gpu->UpdateResolutionScale();
}

void System::SetTimerResolutionIncreased(bool enabled)
{
#if defined(_WIN32)
  static bool current_state = false;
  if (current_state == enabled)
    return;

  current_state = enabled;

  if (enabled)
    timeBeginPeriod(1);
  else
    timeEndPeriod(1);
#endif
}
