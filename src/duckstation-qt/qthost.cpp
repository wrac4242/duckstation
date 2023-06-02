﻿// SPDX-FileCopyrightText: 2019-2022 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: (GPL-3.0 OR CC-BY-NC-ND-4.0)

#include "qthost.h"
#include "common/assert.h"
#include "common/byte_stream.h"
#include "common/crash_handler.h"
#include "common/file_system.h"
#include "common/log.h"
#include "common/path.h"
#include "common/string_util.h"
#include "common/window_info.h"
#include "core/cheats.h"
#include "core/controller.h"
#include "core/game_database.h"
#include "core/gpu.h"
#include "core/host.h"
#include "core/host_settings.h"
#include "core/memory_card.h"
#include "core/spu.h"
#include "core/system.h"
#include "displaywidget.h"
#include "frontend-common/fullscreen_ui.h"
#include "frontend-common/game_list.h"
#include "frontend-common/imgui_manager.h"
#include "frontend-common/imgui_overlays.h"
#include "frontend-common/input_manager.h"
#include "imgui.h"
#include "mainwindow.h"
#include "qtprogresscallback.h"
#include "qtutils.h"
#include "scmversion/scmversion.h"
#include "util/audio_stream.h"
#include "util/ini_settings_interface.h"
#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDebug>
#include <QtCore/QEventLoop>
#include <QtCore/QFile>
#include <QtCore/QTimer>
#include <QtGui/QClipboard>
#include <QtGui/QKeyEvent>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMessageBox>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <memory>
Log_SetChannel(EmuThread);

#ifdef _WIN32
#include "common/windows_headers.h"
#include <ShlObj.h>
#endif

#ifdef WITH_CHEEVOS
#include "frontend-common/achievements.h"
#endif

static constexpr u32 SETTINGS_VERSION = 3;
static constexpr u32 SETTINGS_SAVE_DELAY = 1000;

/// Interval at which the controllers are polled when the system is not active.
static constexpr u32 BACKGROUND_CONTROLLER_POLLING_INTERVAL = 100;

/// Poll at half the vsync rate for FSUI to reduce the chance of getting a press+release in the same frame.
static constexpr u32 FULLSCREEN_UI_CONTROLLER_POLLING_INTERVAL = 8;

//////////////////////////////////////////////////////////////////////////
// Local function declarations
//////////////////////////////////////////////////////////////////////////
namespace QtHost {
static void RegisterTypes();
static bool InitializeConfig(std::string settings_filename);
static bool ShouldUsePortableMode();
static void SetAppRoot();
static void SetResourcesDirectory();
static void SetDataDirectory();
static bool SetCriticalFolders();
static void SetDefaultSettings(SettingsInterface& si, bool system, bool controller);
static void SaveSettings();
static void InitializeEarlyConsole();
static void HookSignals();
static void PrintCommandLineVersion();
static void PrintCommandLineHelp(const char* progname);
static bool ParseCommandLineParametersAndInitializeConfig(QApplication& app,
                                                          std::shared_ptr<SystemBootParameters>& boot_params);
} // namespace QtHost

static std::unique_ptr<INISettingsInterface> s_base_settings_interface;
static std::unique_ptr<QTimer> s_settings_save_timer;
static bool s_batch_mode = false;
static bool s_nogui_mode = false;
static bool s_start_fullscreen_ui = false;
static bool s_start_fullscreen_ui_fullscreen = false;

EmuThread* g_emu_thread;
GDBServer* g_gdb_server;

EmuThread::EmuThread(QThread* ui_thread) : QThread(), m_ui_thread(ui_thread) {}

EmuThread::~EmuThread() = default;

void QtHost::RegisterTypes()
{
  // Register any standard types we need elsewhere
  qRegisterMetaType<std::optional<bool>>();
  qRegisterMetaType<std::function<void()>>("std::function<void()>");
  qRegisterMetaType<std::shared_ptr<SystemBootParameters>>();
  qRegisterMetaType<const GameList::Entry*>();
  qRegisterMetaType<GPURenderer>("GPURenderer");
  qRegisterMetaType<InputBindingKey>("InputBindingKey");
}

bool QtHost::InBatchMode()
{
  return s_batch_mode;
}

bool QtHost::InNoGUIMode()
{
  return s_nogui_mode;
}

QString QtHost::GetAppNameAndVersion()
{
  return QStringLiteral("DuckStation %1 (%2)").arg(g_scm_tag_str).arg(g_scm_branch_str);
}

QString QtHost::GetAppConfigSuffix()
{
#if defined(_DEBUGFAST)
  return QStringLiteral(" [DebugFast]");
#elif defined(_DEBUG)
  return QStringLiteral(" [Debug]");
#else
  return QString();
#endif
}

QString QtHost::GetResourcesBasePath()
{
  return QString::fromStdString(EmuFolders::Resources);
}

bool QtHost::InitializeConfig(std::string settings_filename)
{
  if (!SetCriticalFolders())
    return false;

  if (settings_filename.empty())
    settings_filename = Path::Combine(EmuFolders::DataRoot, "settings.ini");

  Log_InfoPrintf("Loading config from %s.", settings_filename.c_str());
  s_base_settings_interface = std::make_unique<INISettingsInterface>(std::move(settings_filename));
  Host::Internal::SetBaseSettingsLayer(s_base_settings_interface.get());

  uint settings_version;
  if (!s_base_settings_interface->Load() ||
      !s_base_settings_interface->GetUIntValue("Main", "SettingsVersion", &settings_version) ||
      settings_version != SETTINGS_VERSION)
  {
    if (s_base_settings_interface->ContainsValue("Main", "SettingsVersion"))
    {
      // NOTE: No point translating this, because there's no config loaded, so no language loaded.
      Host::ReportErrorAsync("Error", fmt::format("Settings version {} does not match expected version {}, resetting.",
                                                  settings_version, SETTINGS_VERSION));
    }

    s_base_settings_interface->SetUIntValue("Main", "SettingsVersion", SETTINGS_VERSION);
    s_base_settings_interface->SetBoolValue("ControllerPorts", "ControllerSettingsMigrated", true);
    SetDefaultSettings(*s_base_settings_interface, true, true);
    s_base_settings_interface->Save();
  }

  EmuFolders::LoadConfig(*s_base_settings_interface.get());
  EmuFolders::EnsureFoldersExist();

  // We need to create the console window early, otherwise it appears behind the main window.
  if (!Log::IsConsoleOutputEnabled() &&
      s_base_settings_interface->GetBoolValue("Logging", "LogToConsole", Settings::DEFAULT_LOG_TO_CONSOLE))
  {
    Log::SetConsoleOutputParams(true, nullptr, LOGLEVEL_NONE);
  }

  // TEMPORARY: Migrate controller settings to new interface.
  if (!s_base_settings_interface->GetBoolValue("ControllerPorts", "ControllerSettingsMigrated", false))
  {
    s_base_settings_interface->SetBoolValue("ControllerPorts", "ControllerSettingsMigrated", true);
    if (InputManager::MigrateBindings(*s_base_settings_interface.get()))
      s_base_settings_interface->Save();
  }

  InstallTranslator();
  return true;
}

bool QtHost::SetCriticalFolders()
{
  SetAppRoot();
  SetResourcesDirectory();
  SetDataDirectory();

  // logging of directories in case something goes wrong super early
  Log_DevPrintf("AppRoot Directory: %s", EmuFolders::AppRoot.c_str());
  Log_DevPrintf("DataRoot Directory: %s", EmuFolders::DataRoot.c_str());
  Log_DevPrintf("Resources Directory: %s", EmuFolders::Resources.c_str());

  // Write crash dumps to the data directory, since that'll be accessible for certain.
  CrashHandler::SetWriteDirectory(EmuFolders::DataRoot);

  // the resources directory should exist, bail out if not
  if (!FileSystem::DirectoryExists(EmuFolders::Resources.c_str()))
  {
    QMessageBox::critical(nullptr, QStringLiteral("Error"),
                          QStringLiteral("Resources directory is missing, your installation is incomplete."));
    return false;
  }

  return true;
}

bool QtHost::ShouldUsePortableMode()
{
  // Check whether portable.ini exists in the program directory.
  return (FileSystem::FileExists(Path::Combine(EmuFolders::AppRoot, "portable.txt").c_str()) ||
          FileSystem::FileExists(Path::Combine(EmuFolders::AppRoot, "settings.ini").c_str()));
}

void QtHost::SetAppRoot()
{
  std::string program_path(FileSystem::GetProgramPath());
  Log_InfoPrintf("Program Path: %s", program_path.c_str());

  EmuFolders::AppRoot = Path::Canonicalize(Path::GetDirectory(program_path));
}

void QtHost::SetResourcesDirectory()
{
#ifndef __APPLE__
  // On Windows/Linux, these are in the binary directory.
  EmuFolders::Resources = Path::Combine(EmuFolders::AppRoot, "resources");
#else
  // On macOS, this is in the bundle resources directory.
  EmuFolders::Resources = Path::Canonicalize(Path::Combine(EmuFolders::AppRoot, "../Resources"));
#endif
}

void QtHost::SetDataDirectory()
{
  // Already set, e.g. by -portable.
  if (!EmuFolders::DataRoot.empty())
    return;

  if (ShouldUsePortableMode())
  {
    EmuFolders::DataRoot = EmuFolders::AppRoot;
    return;
  }

#if defined(_WIN32)
  // On Windows, use My Documents\DuckStation.
  PWSTR documents_directory;
  if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, NULL, &documents_directory)))
  {
    if (std::wcslen(documents_directory) > 0)
      EmuFolders::DataRoot = Path::Combine(StringUtil::WideStringToUTF8String(documents_directory), "DuckStation");
    CoTaskMemFree(documents_directory);
  }
#elif defined(__linux__)
  // Use $XDG_CONFIG_HOME/duckstation if it exists.
  const char* xdg_config_home = getenv("XDG_CONFIG_HOME");
  if (xdg_config_home && Path::IsAbsolute(xdg_config_home))
  {
    EmuFolders::DataRoot = Path::Combine(xdg_config_home, "duckstation");
  }
  else
  {
    // Use ~/.local/share/duckstation otherwise.
    const char* home_dir = getenv("HOME");
    if (home_dir)
    {
      // ~/.local/share should exist, but just in case it doesn't and this is a fresh profile..
      const std::string local_dir(Path::Combine(home_dir, ".local"));
      const std::string share_dir(Path::Combine(local_dir, "share"));
      FileSystem::EnsureDirectoryExists(local_dir.c_str(), false);
      FileSystem::EnsureDirectoryExists(share_dir.c_str(), false);
      EmuFolders::DataRoot = Path::Combine(share_dir, "duckstation");
    }
  }
#elif defined(__APPLE__)
  static constexpr char MAC_DATA_DIR[] = "Library/Application Support/DuckStation";
  const char* home_dir = getenv("HOME");
  if (home_dir)
    EmuFolders::DataRoot = Path::Combine(home_dir, MAC_DATA_DIR);
#endif

  // make sure it exists
  if (!EmuFolders::DataRoot.empty() && !FileSystem::DirectoryExists(EmuFolders::DataRoot.c_str()))
  {
    // we're in trouble if we fail to create this directory... but try to hobble on with portable
    if (!FileSystem::EnsureDirectoryExists(EmuFolders::DataRoot.c_str(), false))
      EmuFolders::DataRoot.clear();
  }

  // couldn't determine the data directory? fallback to portable.
  if (EmuFolders::DataRoot.empty())
    EmuFolders::DataRoot = EmuFolders::AppRoot;
}

void Host::LoadSettings(SettingsInterface& si, std::unique_lock<std::mutex>& lock)
{
  CommonHost::LoadSettings(si, lock);
  g_emu_thread->loadSettings(si);
}

void EmuThread::loadSettings(SettingsInterface& si)
{
  //
}

void EmuThread::setInitialState(std::optional<bool> override_fullscreen)
{
  m_is_fullscreen = override_fullscreen.value_or(Host::GetBaseBoolSettingValue("Main", "StartFullscreen", false));
  m_is_rendering_to_main = shouldRenderToMain();
  m_is_surfaceless = false;
}

void Host::CheckForSettingsChanges(const Settings& old_settings)
{
  CommonHost::CheckForSettingsChanges(old_settings);
  g_emu_thread->checkForSettingsChanges(old_settings);
}

void EmuThread::checkForSettingsChanges(const Settings& old_settings)
{
  const bool render_to_main = shouldRenderToMain();
  if (m_is_rendering_to_main != render_to_main)
  {
    m_is_rendering_to_main = render_to_main;
    updateDisplayState();
  }

  QMetaObject::invokeMethod(g_main_window, &MainWindow::checkForSettingChanges, Qt::QueuedConnection);
}

void EmuThread::setDefaultSettings(bool system /* = true */, bool controller /* = true */)
{
  if (isOnThread())
  {
    QMetaObject::invokeMethod(this, "setDefaultSettings", Qt::QueuedConnection);
    return;
  }

  auto lock = Host::GetSettingsLock();
  QtHost::SetDefaultSettings(*s_base_settings_interface, system, controller);
  QtHost::QueueSettingsSave();

  applySettings(false);

  if (system)
    emit settingsResetToDefault();
}

void QtHost::SetDefaultSettings(SettingsInterface& si, bool system, bool controller)
{
  if (system)
  {
    System::SetDefaultSettings(si);
    CommonHost::SetDefaultSettings(si);
    EmuFolders::SetDefaults();
    EmuFolders::Save(si);
  }

  if (controller)
  {
    CommonHost::SetDefaultControllerSettings(si);
    CommonHost::SetDefaultHotkeyBindings(si);
  }
}

bool EmuThread::shouldRenderToMain() const
{
  return !Host::GetBoolSettingValue("Main", "RenderToSeparateWindow", false) && !QtHost::InNoGUIMode();
}

void Host::RequestResizeHostDisplay(s32 new_window_width, s32 new_window_height)
{
  if (g_emu_thread->isFullscreen())
    return;

  emit g_emu_thread->displaySizeRequested(new_window_width, new_window_height);
}

void EmuThread::applySettings(bool display_osd_messages /* = false */)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "applySettings", Qt::QueuedConnection, Q_ARG(bool, display_osd_messages));
    return;
  }

  System::ApplySettings(display_osd_messages);
  if (!FullscreenUI::IsInitialized() && System::IsPaused())
    redrawDisplayWindow();
}

void EmuThread::reloadGameSettings(bool display_osd_messages /* = false */)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "reloadGameSettings", Qt::QueuedConnection, Q_ARG(bool, display_osd_messages));
    return;
  }

  System::ReloadGameSettings(display_osd_messages);
  if (!FullscreenUI::IsInitialized() && System::IsPaused())
    redrawDisplayWindow();
}

void EmuThread::updateEmuFolders()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::updateEmuFolders, Qt::QueuedConnection);
    return;
  }

  EmuFolders::Update();
}

void EmuThread::startFullscreenUI()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::startFullscreenUI, Qt::QueuedConnection);
    return;
  }

  if (System::IsValid())
    return;

  // we want settings loaded so we choose the correct renderer
  // this also sorts out input sources.
  System::LoadSettings(false);
  setInitialState(s_start_fullscreen_ui_fullscreen ? std::optional<bool>(true) : std::optional<bool>());
  m_run_fullscreen_ui = true;

  if (!acquireHostDisplay(Settings::GetRenderAPIForRenderer(g_settings.gpu_renderer)))
  {
    m_run_fullscreen_ui = false;
    return;
  }

  // poll more frequently so we don't lose events
  stopBackgroundControllerPollTimer();
  startBackgroundControllerPollTimer();
  wakeThread();
}

void EmuThread::stopFullscreenUI()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::stopFullscreenUI, Qt::QueuedConnection);

    // wait until the host display is gone
    while (g_host_display)
      QApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 1);

    return;
  }

  if (System::IsValid())
    shutdownSystem();

  if (!g_host_display)
    return;

  m_run_fullscreen_ui = false;
  releaseHostDisplay();
}

void EmuThread::bootSystem(std::shared_ptr<SystemBootParameters> params)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "bootSystem", Qt::QueuedConnection,
                              Q_ARG(std::shared_ptr<SystemBootParameters>, std::move(params)));
    return;
  }

  setInitialState(params->override_fullscreen);

  if (!System::BootSystem(std::move(*params)))
    return;

  // force a frame to be drawn to repaint the window
  renderDisplay(false);
}

void EmuThread::bootOrLoadState(std::string path)
{
  DebugAssert(isOnThread());

  if (System::IsValid())
  {
    System::LoadState(path.c_str());
  }
  else
  {
    std::shared_ptr<SystemBootParameters> params = std::make_shared<SystemBootParameters>();
    params->save_state = std::move(path);
    bootSystem(std::move(params));
  }
}

void EmuThread::resumeSystemFromMostRecentState()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::resumeSystemFromMostRecentState, Qt::QueuedConnection);
    return;
  }

  // shouldn't be doing this with a system running
  if (System::IsValid())
    return;

  std::string state_filename(System::GetMostRecentResumeSaveStatePath());
  if (state_filename.empty())
  {
    emit errorReported(tr("Error"), tr("No resume save state found."));
    return;
  }

  bootOrLoadState(std::move(state_filename));
}

void EmuThread::onDisplayWindowKeyEvent(int key, bool pressed)
{
  DebugAssert(isOnThread());

  InputManager::InvokeEvents(InputManager::MakeHostKeyboardKey(key), static_cast<float>(pressed),
                             GenericInputBinding::Unknown);
}

void EmuThread::onDisplayWindowTextEntered(const QString& text)
{
  DebugAssert(isOnThread());

  ImGuiManager::AddTextInput(text.toStdString());
}

void EmuThread::onDisplayWindowMouseMoveEvent(bool relative, float x, float y)
{
  // display might be null here if the event happened after shutdown
  DebugAssert(isOnThread());
  if (!relative)
  {
    if (g_host_display)
      g_host_display->SetMousePosition(static_cast<s32>(x), static_cast<s32>(y));

    InputManager::UpdatePointerAbsolutePosition(0, x, y);
    ImGuiManager::UpdateMousePosition(x, y);
  }
  else
  {
    if (x != 0.0f)
      InputManager::UpdatePointerRelativeDelta(0, InputPointerAxis::X, x);
    if (y != 0.0f)
      InputManager::UpdatePointerRelativeDelta(0, InputPointerAxis::Y, y);

    if (g_host_display)
    {
      const float abs_x = static_cast<float>(g_host_display->GetMousePositionX()) + x;
      const float abs_y = static_cast<float>(g_host_display->GetMousePositionY()) + y;
      g_host_display->SetMousePosition(static_cast<s32>(abs_x), static_cast<s32>(abs_y));
      ImGuiManager::UpdateMousePosition(abs_x, abs_y);
    }
  }
}

void EmuThread::onDisplayWindowMouseButtonEvent(int button, bool pressed)
{
  DebugAssert(isOnThread());

  InputManager::InvokeEvents(InputManager::MakePointerButtonKey(0, button), static_cast<float>(pressed),
                             GenericInputBinding::Unknown);
}

void EmuThread::onDisplayWindowMouseWheelEvent(const QPoint& delta_angle)
{
  DebugAssert(isOnThread());

  const float dx = std::clamp(static_cast<float>(delta_angle.x()) / QtUtils::MOUSE_WHEEL_DELTA, -1.0f, 1.0f);
  if (dx != 0.0f)
    InputManager::UpdatePointerRelativeDelta(0, InputPointerAxis::WheelX, dx);

  const float dy = std::clamp(static_cast<float>(delta_angle.y()) / QtUtils::MOUSE_WHEEL_DELTA, -1.0f, 1.0f);
  if (dy != 0.0f)
    InputManager::UpdatePointerRelativeDelta(0, InputPointerAxis::WheelY, dy);
}

void EmuThread::onDisplayWindowResized(int width, int height)
{
  // this can be null if it was destroyed and the main thread is late catching up
  if (!g_host_display)
    return;

  Log_DevPrintf("Display window resized to %dx%d", width, height);
  g_host_display->ResizeWindow(width, height);
  ImGuiManager::WindowResized();
  System::HostDisplayResized();

  // re-render the display, since otherwise it will be out of date and stretched if paused
  if (System::IsValid())
  {
    if (m_is_exclusive_fullscreen && !g_host_display->IsFullscreen())
    {
      // we lost exclusive fullscreen, switch to borderless
      Host::AddOSDMessage(Host::TranslateStdString("OSDMessage", "Lost exclusive fullscreen."), 10.0f);
      m_is_exclusive_fullscreen = false;
      m_is_fullscreen = false;
      m_lost_exclusive_fullscreen = true;
    }

    // force redraw if we're paused
    if (!System::IsRunning() && !FullscreenUI::HasActiveWindow())
      renderDisplay(false);
  }
}

void EmuThread::redrawDisplayWindow()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "redrawDisplayWindow", Qt::QueuedConnection);
    return;
  }

  if (!g_host_display || System::IsShutdown())
    return;

  renderDisplay(false);
}

void EmuThread::toggleFullscreen()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "toggleFullscreen", Qt::QueuedConnection);
    return;
  }

  setFullscreen(!m_is_fullscreen);
}

void EmuThread::setFullscreen(bool fullscreen)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setFullscreen", Qt::QueuedConnection, Q_ARG(bool, fullscreen));
    return;
  }

  if (!g_host_display || m_is_fullscreen == fullscreen)
    return;

  m_is_fullscreen = fullscreen;
  updateDisplayState();
}

bool Host::IsFullscreen()
{
  return g_emu_thread->isFullscreen();
}

void Host::SetFullscreen(bool enabled)
{
  g_emu_thread->setFullscreen(enabled);
}

void EmuThread::setSurfaceless(bool surfaceless)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setSurfaceless", Qt::QueuedConnection, Q_ARG(bool, surfaceless));
    return;
  }

  if (!g_host_display || m_is_surfaceless == surfaceless)
    return;

  m_is_surfaceless = surfaceless;
  updateDisplayState();
}

void EmuThread::requestDisplaySize(float scale)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "requestDisplaySize", Qt::QueuedConnection, Q_ARG(float, scale));
    return;
  }

  if (!System::IsValid())
    return;

  System::RequestDisplaySize(scale);
}

bool EmuThread::acquireHostDisplay(RenderAPI api)
{
  if (g_host_display)
  {
    if (g_host_display->GetRenderAPI() == api)
    {
      // current is fine
      return true;
    }

    // otherwise we need to switch
    releaseHostDisplay();
  }

  g_host_display = Host::CreateDisplayForAPI(api);
  if (!g_host_display)
    return false;

  if (!createDisplayRequested(m_is_fullscreen, m_is_rendering_to_main))
  {
    emit destroyDisplayRequested();
    g_host_display.reset();
    return false;
  }

  if (!g_host_display->MakeCurrent() || !g_host_display->SetupDevice() || !ImGuiManager::Initialize() ||
      !CommonHost::CreateHostDisplayResources())
  {
    ImGuiManager::Shutdown();
    CommonHost::ReleaseHostDisplayResources();
    g_host_display.reset();
    emit destroyDisplayRequested();
    return false;
  }

  m_is_exclusive_fullscreen = g_host_display->IsFullscreen();

  if (m_run_fullscreen_ui && !FullscreenUI::Initialize())
  {
    Log_ErrorPrint("Failed to initialize fullscreen UI");
    releaseHostDisplay();
    m_run_fullscreen_ui = false;
    return false;
  }

  return true;
}

void EmuThread::connectDisplaySignals(DisplayWidget* widget)
{
  widget->disconnect(this);

  connect(widget, &DisplayWidget::windowResizedEvent, this, &EmuThread::onDisplayWindowResized);
  connect(widget, &DisplayWidget::windowRestoredEvent, this, &EmuThread::redrawDisplayWindow);
  connect(widget, &DisplayWidget::windowKeyEvent, this, &EmuThread::onDisplayWindowKeyEvent);
  connect(widget, &DisplayWidget::windowTextEntered, this, &EmuThread::onDisplayWindowTextEntered);
  connect(widget, &DisplayWidget::windowMouseMoveEvent, this, &EmuThread::onDisplayWindowMouseMoveEvent);
  connect(widget, &DisplayWidget::windowMouseButtonEvent, this, &EmuThread::onDisplayWindowMouseButtonEvent);
  connect(widget, &DisplayWidget::windowMouseWheelEvent, this, &EmuThread::onDisplayWindowMouseWheelEvent);
}

void EmuThread::updateDisplayState()
{
  if (!g_host_display)
    return;

  // this expects the context to get moved back to us afterwards
  g_host_display->DoneCurrent();

  updateDisplayRequested(m_is_fullscreen, m_is_rendering_to_main && !m_is_fullscreen, m_is_surfaceless);
  if (!g_host_display->MakeCurrent())
    Panic("Failed to make device context current after updating");

  m_is_exclusive_fullscreen = g_host_display->IsFullscreen();
  ImGuiManager::WindowResized();
  System::HostDisplayResized();

  if (!System::IsShutdown())
  {
    System::UpdateSoftwareCursor();

    if (!FullscreenUI::IsInitialized() || System::IsPaused())
      redrawDisplayWindow();
  }

  System::UpdateSpeedLimiterState();
}

void EmuThread::releaseHostDisplay()
{
  if (!g_host_display)
    return;

  CommonHost::ReleaseHostDisplayResources();
  ImGuiManager::Shutdown();
  g_host_display.reset();
  emit destroyDisplayRequested();
  m_is_fullscreen = false;
}

void Host::OnSystemStarting()
{
  CommonHost::OnSystemStarting();

  emit g_emu_thread->systemStarting();
}

void Host::OnSystemStarted()
{
  CommonHost::OnSystemStarted();

  g_emu_thread->wakeThread();
  g_emu_thread->stopBackgroundControllerPollTimer();

  emit g_emu_thread->systemStarted();
}

void Host::OnSystemPaused()
{
  CommonHost::OnSystemPaused();

  emit g_emu_thread->systemPaused();
  g_emu_thread->startBackgroundControllerPollTimer();
  g_emu_thread->renderDisplay(false);
}

void Host::OnSystemResumed()
{
  CommonHost::OnSystemResumed();

  // if we were surfaceless (view->game list, system->unpause), get our display widget back
  if (g_emu_thread->isSurfaceless())
    g_emu_thread->setSurfaceless(false);

  emit g_emu_thread->systemResumed();

  g_emu_thread->wakeThread();
  g_emu_thread->stopBackgroundControllerPollTimer();
}

void Host::OnSystemDestroyed()
{
  CommonHost::OnSystemDestroyed();

  g_emu_thread->resetPerformanceCounters();
  g_emu_thread->startBackgroundControllerPollTimer();
  emit g_emu_thread->systemDestroyed();

  // re-wake thread when we're running fsui, otherwise it won't draw
  if (g_emu_thread->isRunningFullscreenUI())
    g_emu_thread->wakeThread();
}

void EmuThread::reloadInputSources()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::reloadInputSources, Qt::QueuedConnection);
    return;
  }

  std::unique_lock<std::mutex> lock = Host::GetSettingsLock();
  SettingsInterface* si = Host::GetSettingsInterface();
  SettingsInterface* bindings_si = Host::GetSettingsInterfaceForBindings();
  InputManager::ReloadSources(*si, lock);
  InputManager::ReloadBindings(*si, *bindings_si);
}

void EmuThread::reloadInputBindings()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::reloadInputBindings, Qt::QueuedConnection);
    return;
  }

  auto lock = Host::GetSettingsLock();
  SettingsInterface* si = Host::GetSettingsInterface();
  SettingsInterface* bindings_si = Host::GetSettingsInterfaceForBindings();
  InputManager::ReloadBindings(*si, *bindings_si);
}

void EmuThread::reloadInputDevices()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::reloadInputDevices, Qt::QueuedConnection);
    return;
  }

  InputManager::ReloadDevices();
}

void EmuThread::closeInputSources()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::reloadInputDevices, Qt::BlockingQueuedConnection);
    return;
  }

  InputManager::CloseSources();
}

void EmuThread::enumerateInputDevices()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::enumerateInputDevices, Qt::QueuedConnection);
    return;
  }

  const std::vector<std::pair<std::string, std::string>> devs(InputManager::EnumerateDevices());
  QList<QPair<QString, QString>> qdevs;
  qdevs.reserve(devs.size());
  for (const std::pair<std::string, std::string>& dev : devs)
    qdevs.emplace_back(QString::fromStdString(dev.first), QString::fromStdString(dev.second));

  onInputDevicesEnumerated(qdevs);
}

void EmuThread::enumerateVibrationMotors()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::enumerateVibrationMotors, Qt::QueuedConnection);
    return;
  }

  const std::vector<InputBindingKey> motors(InputManager::EnumerateMotors());
  QList<InputBindingKey> qmotors;
  qmotors.reserve(motors.size());
  for (InputBindingKey key : motors)
    qmotors.push_back(key);

  onVibrationMotorsEnumerated(qmotors);
}

void EmuThread::shutdownSystem(bool save_state /* = true */)
{
  if (!isOnThread())
  {
    System::CancelPendingStartup();
    QMetaObject::invokeMethod(this, "shutdownSystem", Qt::QueuedConnection, Q_ARG(bool, save_state));
    return;
  }

  System::ShutdownSystem(save_state);
}

void EmuThread::resetSystem()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, &EmuThread::resetSystem, Qt::QueuedConnection);
    return;
  }

  System::ResetSystem();
}

void EmuThread::setSystemPaused(bool paused, bool wait_until_paused /* = false */)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setSystemPaused",
                              wait_until_paused ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
                              Q_ARG(bool, paused), Q_ARG(bool, wait_until_paused));
    return;
  }

  System::PauseSystem(paused);
}

void EmuThread::changeDisc(const QString& new_disc_filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "changeDisc", Qt::QueuedConnection, Q_ARG(const QString&, new_disc_filename));
    return;
  }

  if (System::IsShutdown())
    return;

  if (!new_disc_filename.isEmpty())
    System::InsertMedia(new_disc_filename.toStdString().c_str());
  else
    System::RemoveMedia();
}

void EmuThread::changeDiscFromPlaylist(quint32 index)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "changeDiscFromPlaylist", Qt::QueuedConnection, Q_ARG(quint32, index));
    return;
  }

  if (System::IsShutdown())
    return;

  if (!System::SwitchMediaSubImage(index))
    Host::ReportFormattedErrorAsync("Error", "Failed to switch to subimage %u", index);
}

void EmuThread::loadCheatList(const QString& filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "loadCheatList", Qt::QueuedConnection, Q_ARG(const QString&, filename));
    return;
  }

  System::LoadCheatList(filename.toUtf8().constData());
}

void EmuThread::setCheatEnabled(quint32 index, bool enabled)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setCheatEnabled", Qt::QueuedConnection, Q_ARG(quint32, index),
                              Q_ARG(bool, enabled));
    return;
  }

  System::SetCheatCodeState(index, enabled, g_settings.auto_load_cheats);
  emit cheatEnabled(index, enabled);
}

void EmuThread::applyCheat(quint32 index)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "applyCheat", Qt::QueuedConnection, Q_ARG(quint32, index));
    return;
  }

  System::ApplyCheatCode(index);
}

void EmuThread::reloadPostProcessingShaders()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "reloadPostProcessingShaders", Qt::QueuedConnection);
    return;
  }

  System::ReloadPostProcessingShaders();
}

void EmuThread::clearInputBindStateFromSource(InputBindingKey key)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "clearInputBindStateFromSource", Qt::QueuedConnection, Q_ARG(InputBindingKey, key));
    return;
  }

  InputManager::ClearBindStateFromSource(key);
}

void EmuThread::runOnEmuThread(std::function<void()> callback)
{
  callback();
}

void Host::RunOnCPUThread(std::function<void()> function, bool block /* = false */)
{
  const bool self = g_emu_thread->isOnThread();

  QMetaObject::invokeMethod(g_emu_thread, "runOnEmuThread",
                            (block && !self) ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
                            Q_ARG(std::function<void()>, std::move(function)));
}

void QtHost::RunOnUIThread(const std::function<void()>& func, bool block /*= false*/)
{
  // main window always exists, so it's fine to attach it to that.
  QMetaObject::invokeMethod(g_main_window, "runOnUIThread", block ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
                            Q_ARG(const std::function<void()>&, func));
}

void Host::RefreshGameListAsync(bool invalidate_cache)
{
  QMetaObject::invokeMethod(g_main_window, "refreshGameList", Qt::QueuedConnection, Q_ARG(bool, invalidate_cache));
}

void Host::CancelGameListRefresh()
{
  QMetaObject::invokeMethod(g_main_window, "cancelGameListRefresh", Qt::BlockingQueuedConnection);
}

void EmuThread::loadState(const QString& filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "loadState", Qt::QueuedConnection, Q_ARG(const QString&, filename));
    return;
  }

  bootOrLoadState(filename.toStdString());
}

void EmuThread::loadState(bool global, qint32 slot)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "loadState", Qt::QueuedConnection, Q_ARG(bool, global), Q_ARG(qint32, slot));
    return;
  }

  // shouldn't even get here if we don't have a running game
  if (!global && System::GetGameSerial().empty())
    return;

  bootOrLoadState(global ? System::GetGlobalSaveStateFileName(slot) :
                           System::GetGameSaveStateFileName(System::GetGameSerial(), slot));
}

void EmuThread::saveState(const QString& filename, bool block_until_done /* = false */)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "saveState", block_until_done ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
                              Q_ARG(const QString&, filename), Q_ARG(bool, block_until_done));
    return;
  }

  if (!System::IsValid())
    return;

  System::SaveState(filename.toUtf8().data(), g_settings.create_save_state_backups);
}

void EmuThread::saveState(bool global, qint32 slot, bool block_until_done /* = false */)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "saveState", block_until_done ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
                              Q_ARG(bool, global), Q_ARG(qint32, slot), Q_ARG(bool, block_until_done));
    return;
  }

  if (!global && System::GetGameSerial().empty())
    return;

  System::SaveState((global ? System::GetGlobalSaveStateFileName(slot) :
                              System::GetGameSaveStateFileName(System::GetGameSerial(), slot))
                      .c_str(),
                    g_settings.create_save_state_backups);
}

void EmuThread::undoLoadState()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "undoLoadState", Qt::QueuedConnection);
    return;
  }

  System::UndoLoadState();
}

void EmuThread::setAudioOutputVolume(int volume, int fast_forward_volume)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setAudioOutputVolume", Qt::QueuedConnection, Q_ARG(int, volume),
                              Q_ARG(int, fast_forward_volume));
    return;
  }

  g_settings.audio_output_volume = volume;
  g_settings.audio_fast_forward_volume = fast_forward_volume;
  System::UpdateVolume();
}

void EmuThread::setAudioOutputMuted(bool muted)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "setAudioOutputMuted", Qt::QueuedConnection, Q_ARG(bool, muted));
    return;
  }

  g_settings.audio_output_muted = muted;
  System::UpdateVolume();
}

void EmuThread::startDumpingAudio()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "startDumpingAudio", Qt::QueuedConnection);
    return;
  }

  System::StartDumpingAudio();
}

void EmuThread::stopDumpingAudio()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "stopDumpingAudio", Qt::QueuedConnection);
    return;
  }

  System::StopDumpingAudio();
}

void EmuThread::singleStepCPU()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "singleStepCPU", Qt::BlockingQueuedConnection);
    return;
  }

  if (!System::IsValid())
    return;

  System::SingleStepCPU();
  renderDisplay(false);
}

void EmuThread::dumpRAM(const QString& filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "dumpRAM", Qt::QueuedConnection, Q_ARG(const QString&, filename));
    return;
  }

  const std::string filename_str = filename.toStdString();
  if (System::DumpRAM(filename_str.c_str()))
    Host::AddOSDMessage(fmt::format("RAM dumped to '{}'", filename_str), 10.0f);
  else
    Host::ReportErrorAsync("Error", fmt::format("Failed to dump RAM to '{}'", filename_str));
}

void EmuThread::dumpVRAM(const QString& filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "dumpVRAM", Qt::QueuedConnection, Q_ARG(const QString&, filename));
    return;
  }

  const std::string filename_str = filename.toStdString();
  if (System::DumpVRAM(filename_str.c_str()))
    Host::AddOSDMessage(fmt::format("VRAM dumped to '{}'", filename_str), 10.0f);
  else
    Host::ReportErrorAsync("Error", fmt::format("Failed to dump VRAM to '{}'", filename_str));
}

void EmuThread::dumpSPURAM(const QString& filename)
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "dumpSPURAM", Qt::QueuedConnection, Q_ARG(const QString&, filename));
    return;
  }

  const std::string filename_str = filename.toStdString();
  if (System::DumpSPURAM(filename_str.c_str()))
    Host::AddOSDMessage(fmt::format("SPU RAM dumped to '{}'", filename_str), 10.0f);
  else
    Host::ReportErrorAsync("Error", fmt::format("Failed to dump SPU RAM to '{}'", filename_str));
}

void EmuThread::saveScreenshot()
{
  if (!isOnThread())
  {
    QMetaObject::invokeMethod(this, "saveScreenshot", Qt::QueuedConnection);
    return;
  }

  System::SaveScreenshot(nullptr, true, true);
}

#ifdef WITH_CHEEVOS

void Host::OnAchievementsRefreshed()
{
  u32 game_id = 0;
  u32 achievement_count = 0;
  u32 max_points = 0;

  QString game_info;

  if (Achievements::HasActiveGame())
  {
    game_id = Achievements::GetGameID();
    achievement_count = Achievements::GetAchievementCount();
    max_points = Achievements::GetMaximumPointsForGame();

    game_info = qApp
                  ->translate("EmuThread", "Game ID: %1\n"
                                           "Game Title: %2\n"
                                           "Achievements: %5 (%6)\n\n")
                  .arg(game_id)
                  .arg(QString::fromStdString(Achievements::GetGameTitle()))
                  .arg(achievement_count)
                  .arg(qApp->translate("EmuThread", "%n points", "", max_points));

    const std::string rich_presence_string(Achievements::GetRichPresenceString());
    if (!rich_presence_string.empty())
      game_info.append(QString::fromStdString(rich_presence_string));
    else
      game_info.append(qApp->translate("EmuThread", "Rich presence inactive or unsupported."));
  }
  else
  {
    game_info = qApp->translate("EmuThread", "Game not loaded or no RetroAchievements available.");
  }

  emit g_emu_thread->achievementsRefreshed(game_id, game_info, achievement_count, max_points);
}

void Host::OnAchievementsChallengeModeChanged()
{
  emit g_emu_thread->achievementsChallengeModeChanged();
}

#endif

void EmuThread::doBackgroundControllerPoll()
{
  InputManager::PollSources();
}

void EmuThread::createBackgroundControllerPollTimer()
{
  DebugAssert(!m_background_controller_polling_timer);
  m_background_controller_polling_timer = new QTimer(this);
  m_background_controller_polling_timer->setSingleShot(false);
  m_background_controller_polling_timer->setTimerType(Qt::CoarseTimer);
  connect(m_background_controller_polling_timer, &QTimer::timeout, this, &EmuThread::doBackgroundControllerPoll);
}

void EmuThread::destroyBackgroundControllerPollTimer()
{
  delete m_background_controller_polling_timer;
  m_background_controller_polling_timer = nullptr;
}

void EmuThread::startBackgroundControllerPollTimer()
{
  if (m_background_controller_polling_timer->isActive())
    return;

  m_background_controller_polling_timer->start(
    FullscreenUI::IsInitialized() ? FULLSCREEN_UI_CONTROLLER_POLLING_INTERVAL : BACKGROUND_CONTROLLER_POLLING_INTERVAL);
}

void EmuThread::stopBackgroundControllerPollTimer()
{
  if (!m_background_controller_polling_timer->isActive())
    return;

  m_background_controller_polling_timer->stop();
}

void EmuThread::start()
{
  AssertMsg(!g_emu_thread, "Emu thread does not exist");

  g_emu_thread = new EmuThread(QThread::currentThread());
  g_gdb_server = new GDBServer();
  g_gdb_server->moveToThread(g_emu_thread);
  g_emu_thread->QThread::start();
  g_emu_thread->m_started_semaphore.acquire();
  g_emu_thread->moveToThread(g_emu_thread);
}

void EmuThread::stop()
{
  AssertMsg(g_emu_thread, "Emu thread exists");
  AssertMsg(!g_emu_thread->isOnThread(), "Not called on the emu thread");

  QMetaObject::invokeMethod(g_emu_thread, &EmuThread::stopInThread, Qt::QueuedConnection);
  while (g_emu_thread->isRunning())
    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents, 1);
}

void EmuThread::stopInThread()
{
  stopFullscreenUI();

  m_shutdown_flag = true;
  m_event_loop->quit();
}

void EmuThread::run()
{
  m_event_loop = new QEventLoop();
  m_started_semaphore.release();

  // input source setup must happen on emu thread
  CommonHost::Initialize();

  // bind buttons/axises
  createBackgroundControllerPollTimer();
  startBackgroundControllerPollTimer();

  // main loop
  while (!m_shutdown_flag)
  {
    if (System::IsRunning())
    {
      System::Execute();
    }
    else
    {
      // we want to keep rendering the UI when paused and fullscreen UI is enabled
      if (!FullscreenUI::HasActiveWindow() && !System::IsRunning())
      {
        // wait until we have a system before running
        m_event_loop->exec();
        continue;
      }

      m_event_loop->processEvents(QEventLoop::AllEvents);
      CommonHost::PumpMessagesOnCPUThread();
      if (g_host_display)
      {
        renderDisplay(false);
        if (!g_host_display->IsVsyncEnabled())
          g_host_display->ThrottlePresentation();
      }
    }
  }

  if (System::IsValid())
    System::ShutdownSystem(false);

  destroyBackgroundControllerPollTimer();
  CommonHost::Shutdown();

  // move back to UI thread
  moveToThread(m_ui_thread);
}

void EmuThread::renderDisplay(bool skip_present)
{
  // acquire for IO.MousePos.
  std::atomic_thread_fence(std::memory_order_acquire);

  if (!skip_present)
  {
    FullscreenUI::Render();
    ImGuiManager::RenderTextOverlays();
    ImGuiManager::RenderOSDMessages();
  }

  // Debug windows are always rendered, otherwise mouse input breaks on skip.
  ImGuiManager::RenderOverlayWindows();
  ImGuiManager::RenderDebugWindows();

  g_host_display->Render(skip_present);

  ImGuiManager::NewFrame();
}

void Host::InvalidateDisplay()
{
  g_emu_thread->renderDisplay(false);
}

void Host::RenderDisplay(bool skip_present)
{
  g_emu_thread->renderDisplay(skip_present);
}

void EmuThread::wakeThread()
{
  if (isOnThread())
    m_event_loop->quit();
  else
    QMetaObject::invokeMethod(m_event_loop, "quit", Qt::QueuedConnection);
}

TinyString Host::TranslateString(const char* context, const char* str, const char* disambiguation /*= nullptr*/,
                                 int n /*= -1*/)
{
  const QByteArray bytes(qApp->translate(context, str, disambiguation, n).toUtf8());
  return TinyString(bytes.constData(), bytes.size());
}

std::string Host::TranslateStdString(const char* context, const char* str, const char* disambiguation /*= nullptr*/,
                                     int n /*= -1*/)
{
  return qApp->translate(context, str, disambiguation, n).toStdString();
}

void Host::ReportErrorAsync(const std::string_view& title, const std::string_view& message)
{
  if (!title.empty() && !message.empty())
  {
    Log_ErrorPrintf("ReportErrorAsync: %.*s: %.*s", static_cast<int>(title.size()), title.data(),
                    static_cast<int>(message.size()), message.data());
  }
  else if (!message.empty())
  {
    Log_ErrorPrintf("ReportErrorAsync: %.*s", static_cast<int>(message.size()), message.data());
  }

  QMetaObject::invokeMethod(
    g_main_window, "reportError", Qt::QueuedConnection,
    Q_ARG(const QString&, title.empty() ? QString() : QString::fromUtf8(title.data(), title.size())),
    Q_ARG(const QString&, message.empty() ? QString() : QString::fromUtf8(message.data(), message.size())));
}

bool Host::ConfirmMessage(const std::string_view& title, const std::string_view& message)
{
  auto lock = g_emu_thread->pauseAndLockSystem();

  return emit g_emu_thread->messageConfirmed(QString::fromUtf8(title.data(), title.size()),
                                             QString::fromUtf8(message.data(), message.size()));
}

void Host::OpenURL(const std::string_view& url)
{
  QtHost::RunOnUIThread([url = QtUtils::StringViewToQString(url)]() { QtUtils::OpenURL(g_main_window, QUrl(url)); });
}

bool Host::CopyTextToClipboard(const std::string_view& text)
{
  QtHost::RunOnUIThread([text = QtUtils::StringViewToQString(text)]() {
    QClipboard* clipboard = QGuiApplication::clipboard();
    if (clipboard)
      clipboard->setText(text);
  });
  return true;
}

void Host::ReportDebuggerMessage(const std::string_view& message)
{
  emit g_emu_thread->debuggerMessageReported(QString::fromUtf8(message));
}

void Host::OnInputDeviceConnected(const std::string_view& identifier, const std::string_view& device_name)
{
  emit g_emu_thread->onInputDeviceConnected(
    identifier.empty() ? QString() : QString::fromUtf8(identifier.data(), identifier.size()),
    device_name.empty() ? QString() : QString::fromUtf8(device_name.data(), device_name.size()));
}

void Host::OnInputDeviceDisconnected(const std::string_view& identifier)
{
  emit g_emu_thread->onInputDeviceDisconnected(
    identifier.empty() ? QString() : QString::fromUtf8(identifier.data(), identifier.size()));
}

std::optional<std::vector<u8>> Host::ReadResourceFile(const char* filename)
{
  const std::string path(Path::Combine(EmuFolders::Resources, filename));
  std::optional<std::vector<u8>> ret(FileSystem::ReadBinaryFile(path.c_str()));
  if (!ret.has_value())
    Log_ErrorPrintf("Failed to read resource file '%s'", filename);
  return ret;
}

std::optional<std::string> Host::ReadResourceFileToString(const char* filename)
{
  const std::string path(Path::Combine(EmuFolders::Resources, filename));
  std::optional<std::string> ret(FileSystem::ReadFileToString(path.c_str()));
  if (!ret.has_value())
    Log_ErrorPrintf("Failed to read resource file to string '%s'", filename);
  return ret;
}

std::optional<std::time_t> Host::GetResourceFileTimestamp(const char* filename)
{
  const std::string path(Path::Combine(EmuFolders::Resources, filename));
  FILESYSTEM_STAT_DATA sd;
  if (!FileSystem::StatFile(path.c_str(), &sd))
  {
    Log_ErrorPrintf("Failed to stat resource file '%s'", filename);
    return std::nullopt;
  }

  return sd.ModificationTime;
}

void Host::CommitBaseSettingChanges()
{
  if (g_emu_thread->isOnThread())
    QtHost::RunOnUIThread([]() { QtHost::QueueSettingsSave(); });
  else
    QtHost::QueueSettingsSave();
}

bool Host::AcquireHostDisplay(RenderAPI api)
{
  return g_emu_thread->acquireHostDisplay(api);
}

void Host::ReleaseHostDisplay()
{
  if (g_emu_thread->isRunningFullscreenUI())
  {
    // keep display alive when running fsui
    return;
  }

  g_emu_thread->releaseHostDisplay();
}

void EmuThread::updatePerformanceCounters()
{
  GPURenderer renderer = GPURenderer::Count;
  u32 render_width = 0;
  u32 render_height = 0;

  if (g_gpu)
  {
    renderer = g_gpu->GetRendererType();
    std::tie(render_width, render_height) = g_gpu->GetEffectiveDisplayResolution();
  }

  if (renderer != m_last_renderer)
  {
    QMetaObject::invokeMethod(g_main_window->getStatusRendererWidget(), "setText", Qt::QueuedConnection,
                              Q_ARG(const QString&, QString::fromUtf8(Settings::GetRendererName(renderer))));
    m_last_renderer = renderer;
  }
  if (render_width != m_last_render_width || render_height != m_last_render_height)
  {
    QMetaObject::invokeMethod(g_main_window->getStatusResolutionWidget(), "setText", Qt::QueuedConnection,
                              Q_ARG(const QString&, tr("%1x%2").arg(render_width).arg(render_height)));
    m_last_render_width = render_width;
    m_last_render_height = render_height;
  }

  const float gfps = System::GetFPS();
  if (gfps != m_last_game_fps)
  {
    QMetaObject::invokeMethod(g_main_window->getStatusFPSWidget(), "setText", Qt::QueuedConnection,
                              Q_ARG(const QString&, tr("Game: %1 FPS").arg(gfps, 0, 'f', 0)));
    m_last_game_fps = gfps;
  }

  const float speed = System::GetEmulationSpeed();
  const float vfps = System::GetVPS();
  if (speed != m_last_speed || vfps != m_last_video_fps)
  {
    QMetaObject::invokeMethod(
      g_main_window->getStatusVPSWidget(), "setText", Qt::QueuedConnection,
      Q_ARG(const QString&, tr("Video: %1 FPS (%2%)").arg(vfps, 0, 'f', 0).arg(speed, 0, 'f', 0)));
    m_last_speed = speed;
    m_last_video_fps = vfps;
  }
}

void EmuThread::resetPerformanceCounters()
{
  m_last_speed = std::numeric_limits<float>::infinity();
  m_last_game_fps = std::numeric_limits<float>::infinity();
  m_last_video_fps = std::numeric_limits<float>::infinity();
  m_last_render_width = std::numeric_limits<u32>::max();
  m_last_render_height = std::numeric_limits<u32>::max();
  m_last_renderer = GPURenderer::Count;

  QString blank;
  QMetaObject::invokeMethod(g_main_window->getStatusRendererWidget(), "setText", Qt::QueuedConnection,
                            Q_ARG(const QString&, blank));
  QMetaObject::invokeMethod(g_main_window->getStatusResolutionWidget(), "setText", Qt::QueuedConnection,
                            Q_ARG(const QString&, blank));
  QMetaObject::invokeMethod(g_main_window->getStatusFPSWidget(), "setText", Qt::QueuedConnection,
                            Q_ARG(const QString&, blank));
  QMetaObject::invokeMethod(g_main_window->getStatusVPSWidget(), "setText", Qt::QueuedConnection,
                            Q_ARG(const QString&, blank));
}

void Host::OnPerformanceCountersUpdated()
{
  g_emu_thread->updatePerformanceCounters();
}

void Host::OnGameChanged(const std::string& disc_path, const std::string& game_serial, const std::string& game_name)
{
  CommonHost::OnGameChanged(disc_path, game_serial, game_name);

  emit g_emu_thread->runningGameChanged(QString::fromStdString(disc_path), QString::fromStdString(game_serial),
                                        QString::fromStdString(game_name));
}

void Host::SetMouseMode(bool relative, bool hide_cursor)
{
  // TODO: Find a better home for this.
  if (InputManager::HasPointerAxisBinds())
  {
    relative = true;
    hide_cursor = true;
  }

  emit g_emu_thread->mouseModeRequested(relative, hide_cursor);
}

void Host::PumpMessagesOnCPUThread()
{
  g_emu_thread->getEventLoop()->processEvents(QEventLoop::AllEvents);
  CommonHost::PumpMessagesOnCPUThread(); // calls InputManager::PollSources()
}

void QtHost::SaveSettings()
{
  AssertMsg(!g_emu_thread->isOnThread(), "Saving should happen on the UI thread.");

  {
    auto lock = Host::GetSettingsLock();
    if (!s_base_settings_interface->Save())
      Log_ErrorPrintf("Failed to save settings.");
  }

  s_settings_save_timer->deleteLater();
  s_settings_save_timer.release();
}

void QtHost::QueueSettingsSave()
{
  if (g_emu_thread->isOnThread())
  {
    QtHost::RunOnUIThread(QueueSettingsSave);
    return;
  }

  if (s_settings_save_timer)
    return;

  s_settings_save_timer = std::make_unique<QTimer>();
  s_settings_save_timer->connect(s_settings_save_timer.get(), &QTimer::timeout, SaveSettings);
  s_settings_save_timer->setSingleShot(true);
  s_settings_save_timer->start(SETTINGS_SAVE_DELAY);
}

void Host::RequestSystemShutdown(bool allow_confirm, bool save_state)
{
  if (!System::IsValid())
    return;

  QMetaObject::invokeMethod(g_main_window, "requestShutdown", Qt::QueuedConnection, Q_ARG(bool, allow_confirm),
                            Q_ARG(bool, true), Q_ARG(bool, save_state));
}

void Host::RequestExit(bool allow_confirm)
{
  QMetaObject::invokeMethod(g_main_window, "requestExit", Qt::QueuedConnection, Q_ARG(bool, allow_confirm));
}

std::optional<WindowInfo> Host::GetTopLevelWindowInfo()
{
  // Normally we'd just feed the std::optional all the way through here. But that won't work because of some bug
  // in Qt 6.1, and we can't upgrade that because of raging/abusive Win7 users... to anyone still using that dead
  // OS, this is a passive-aggressive "screw you".
  WindowInfo ret;
  QMetaObject::invokeMethod(g_main_window, "getWindowInfo", Qt::BlockingQueuedConnection, Q_ARG(WindowInfo*, &ret));
  return ret;
}

EmuThread::SystemLock EmuThread::pauseAndLockSystem()
{
  const bool was_fullscreen = System::IsValid() && isFullscreen();
  const bool was_paused = System::IsPaused();

  // We use surfaceless rather than switching out of fullscreen, because
  // we're paused, so we're not going to be rendering anyway.
  if (was_fullscreen)
    setSurfaceless(true);
  if (!was_paused)
    setSystemPaused(true);

  return SystemLock(was_paused, was_fullscreen);
}

EmuThread::SystemLock::SystemLock(bool was_paused, bool was_fullscreen)
  : m_was_paused(was_paused), m_was_fullscreen(was_fullscreen)
{
}

EmuThread::SystemLock::SystemLock(SystemLock&& lock)
  : m_was_paused(lock.m_was_paused), m_was_fullscreen(lock.m_was_fullscreen)
{
  lock.m_was_paused = true;
  lock.m_was_fullscreen = false;
}

EmuThread::SystemLock::~SystemLock()
{
  if (m_was_fullscreen)
    g_emu_thread->setSurfaceless(false);
  if (!m_was_paused)
    g_emu_thread->setSystemPaused(false);
}

void EmuThread::SystemLock::cancelResume()
{
  m_was_paused = true;
  m_was_fullscreen = false;
}

BEGIN_HOTKEY_LIST(g_host_hotkeys)
END_HOTKEY_LIST()

static void SignalHandler(int signal)
{
  // First try the normal (graceful) shutdown/exit.
  static bool graceful_shutdown_attempted = false;
  if (!graceful_shutdown_attempted && g_main_window)
  {
    std::fprintf(stderr, "Received CTRL+C, attempting graceful shutdown. Press CTRL+C again to force.\n");
    graceful_shutdown_attempted = true;

    // This could be a bit risky invoking from a signal handler... hopefully it's okay.
    QMetaObject::invokeMethod(g_main_window, "requestExit", Qt::QueuedConnection, Q_ARG(bool, true));
    return;
  }

  std::signal(signal, SIG_DFL);

  // MacOS is missing std::quick_exit() despite it being C++11...
#ifndef __APPLE__
  std::quick_exit(1);
#else
  _Exit(1);
#endif
}

void QtHost::HookSignals()
{
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

#ifdef __linux__
  // Ignore SIGCHLD by default on Linux, since we kick off aplay asynchronously.
  struct sigaction sa_chld = {};
  sigemptyset(&sa_chld.sa_mask);
  sa_chld.sa_flags = SA_SIGINFO | SA_RESTART | SA_NOCLDSTOP | SA_NOCLDWAIT;
  sigaction(SIGCHLD, &sa_chld, nullptr);
#endif
}

void QtHost::InitializeEarlyConsole()
{
  const bool was_console_enabled = Log::IsConsoleOutputEnabled();
  if (!was_console_enabled)
    Log::SetConsoleOutputParams(true);
}

void QtHost::PrintCommandLineVersion()
{
  InitializeEarlyConsole();

  std::fprintf(stderr, "DuckStation Version %s (%s)\n", g_scm_tag_str, g_scm_branch_str);
  std::fprintf(stderr, "https://github.com/stenzek/duckstation\n");
  std::fprintf(stderr, "\n");
}

void QtHost::PrintCommandLineHelp(const char* progname)
{
  InitializeEarlyConsole();

  PrintCommandLineVersion();
  std::fprintf(stderr, "Usage: %s [parameters] [--] [boot filename]\n", progname);
  std::fprintf(stderr, "\n");
  std::fprintf(stderr, "  -help: Displays this information and exits.\n");
  std::fprintf(stderr, "  -version: Displays version information and exits.\n");
  std::fprintf(stderr, "  -batch: Enables batch mode (exits after powering off).\n");
  std::fprintf(stderr, "  -fastboot: Force fast boot for provided filename.\n");
  std::fprintf(stderr, "  -slowboot: Force slow boot for provided filename.\n");
  std::fprintf(stderr, "  -bios: Boot into the BIOS shell.\n");
  std::fprintf(stderr, "  -resume: Load resume save state. If a boot filename is provided,\n"
                       "    that game's resume state will be loaded, otherwise the most\n"
                       "    recent resume save state will be loaded.\n");
  std::fprintf(stderr, "  -state <index>: Loads specified save state by index. If a boot\n"
                       "    filename is provided, a per-game state will be loaded, otherwise\n"
                       "    a global state will be loaded.\n");
  std::fprintf(stderr, "  -statefile <filename>: Loads state from the specified filename.\n"
                       "    No boot filename is required with this option.\n");
  std::fprintf(stderr, "  -exe <filename>: Boot the specified exe instead of loading from disc.\n");
  std::fprintf(stderr, "  -fullscreen: Enters fullscreen mode immediately after starting.\n");
  std::fprintf(stderr, "  -nofullscreen: Prevents fullscreen mode from triggering if enabled.\n");
  std::fprintf(stderr, "  -nogui: Disables main window from being shown, exits on shutdown.\n");
  std::fprintf(stderr, "  -bigpicture: Automatically starts big picture UI.\n");
  std::fprintf(stderr, "  -portable: Forces \"portable mode\", data in same directory.\n");
  std::fprintf(stderr, "  -settings <filename>: Loads a custom settings configuration from the\n"
                       "    specified filename. Default settings applied if file not found.\n");
  std::fprintf(stderr, "  -earlyconsole: Creates console as early as possible, for logging.\n");
#ifdef WITH_RAINTEGRATION
  std::fprintf(stderr, "  -raintegration: Use RAIntegration instead of built-in achievement support.\n");
#endif
  std::fprintf(stderr, "  --: Signals that no more arguments will follow and the remaining\n"
                       "    parameters make up the filename. Use when the filename contains\n"
                       "    spaces or starts with a dash.\n");
  std::fprintf(stderr, "\n");
}

std::shared_ptr<SystemBootParameters>& AutoBoot(std::shared_ptr<SystemBootParameters>& autoboot)
{
  if (!autoboot)
    autoboot = std::make_shared<SystemBootParameters>();

  return autoboot;
}

bool QtHost::ParseCommandLineParametersAndInitializeConfig(QApplication& app,
                                                           std::shared_ptr<SystemBootParameters>& autoboot)
{
  const QStringList args(app.arguments());
  std::optional<s32> state_index;
  std::string settings_filename;
  bool starting_bios = false;

  bool no_more_args = false;

  for (qsizetype i = 1; i < args.size(); i++)
  {
    if (!no_more_args)
    {
#define CHECK_ARG(str) (args[i] == QStringLiteral(str))
#define CHECK_ARG_PARAM(str) (args[i] == QStringLiteral(str) && ((i + 1) < args.size()))

      if (CHECK_ARG("-help"))
      {
        PrintCommandLineHelp(args[0].toUtf8().constData());
        return false;
      }
      else if (CHECK_ARG("-version"))
      {
        PrintCommandLineVersion();
        return false;
      }
      else if (CHECK_ARG("-batch"))
      {
        Log_InfoPrintf("Command Line: Using batch mode.");
        s_batch_mode = true;
        continue;
      }
      else if (CHECK_ARG("-nogui"))
      {
        Log_InfoPrintf("Command Line: Using NoGUI mode.");
        s_nogui_mode = true;
        s_batch_mode = true;
        continue;
      }
      else if (CHECK_ARG("-bios"))
      {
        Log_InfoPrintf("Command Line: Starting BIOS.");
        AutoBoot(autoboot);
        starting_bios = true;
        continue;
      }
      else if (CHECK_ARG("-fastboot"))
      {
        Log_InfoPrintf("Command Line: Forcing fast boot.");
        AutoBoot(autoboot)->override_fast_boot = true;
        continue;
      }
      else if (CHECK_ARG("-slowboot"))
      {
        Log_InfoPrintf("Command Line: Forcing slow boot.");
        AutoBoot(autoboot)->override_fast_boot = false;
        continue;
      }
      else if (CHECK_ARG("-resume"))
      {
        state_index = -1;
        Log_InfoPrintf("Command Line: Loading resume state.");
        continue;
      }
      else if (CHECK_ARG_PARAM("-state"))
      {
        state_index = args[++i].toInt();
        Log_InfoPrintf("Command Line: Loading state index: %d", state_index.value());
        continue;
      }
      else if (CHECK_ARG_PARAM("-statefile"))
      {
        AutoBoot(autoboot)->save_state = args[++i].toStdString();
        Log_InfoPrintf("Command Line: Loading state file: '%s'", autoboot->save_state.c_str());
        continue;
      }
      else if (CHECK_ARG_PARAM("-exe"))
      {
        AutoBoot(autoboot)->override_exe = args[++i].toStdString();
        Log_InfoPrintf("Command Line: Overriding EXE file: '%s'", autoboot->override_exe.c_str());
        continue;
      }
      else if (CHECK_ARG("-fullscreen"))
      {
        Log_InfoPrintf("Command Line: Using fullscreen.");
        AutoBoot(autoboot)->override_fullscreen = true;
        s_start_fullscreen_ui_fullscreen = true;
        continue;
      }
      else if (CHECK_ARG("-nofullscreen"))
      {
        Log_InfoPrintf("Command Line: Not using fullscreen.");
        AutoBoot(autoboot)->override_fullscreen = false;
        continue;
      }
      else if (CHECK_ARG("-portable"))
      {
        Log_InfoPrintf("Command Line: Using portable mode.");
        EmuFolders::DataRoot = EmuFolders::AppRoot;
        continue;
      }
      else if (CHECK_ARG_PARAM("-settings"))
      {
        settings_filename = args[++i].toStdString();
        Log_InfoPrintf("Command Line: Overriding settings filename: %s", settings_filename.c_str());
        continue;
      }
      else if (CHECK_ARG("-bigpicture"))
      {
        Log_InfoPrintf("Command Line: Starting big picture mode.");
        s_start_fullscreen_ui = true;
        continue;
      }
      else if (CHECK_ARG("-earlyconsole"))
      {
        InitializeEarlyConsole();
        continue;
      }
#ifdef WITH_RAINTEGRATION
      else if (CHECK_ARG("-raintegration"))
      {
        Achievements::SwitchToRAIntegration();
        continue;
      }
#endif
      else if (CHECK_ARG("--"))
      {
        no_more_args = true;
        continue;
      }
      else if (args[i][0] == QChar('-'))
      {
        QMessageBox::critical(nullptr, QStringLiteral("Error"), QStringLiteral("Unknown parameter: %1").arg(args[i]));
        return false;
      }

#undef CHECK_ARG
#undef CHECK_ARG_PARAM
    }

    if (autoboot && !autoboot->filename.empty())
      autoboot->filename += ' ';
    AutoBoot(autoboot)->filename += args[i].toStdString();
  }

  // To do anything useful, we need the config initialized.
  if (!QtHost::InitializeConfig(std::move(settings_filename)))
  {
    // NOTE: No point translating this, because no config means the language won't be loaded anyway.
    QMessageBox::critical(nullptr, QStringLiteral("Error"), QStringLiteral("Failed to initialize config."));
    return EXIT_FAILURE;
  }

  // Check the file we're starting actually exists.

  if (autoboot && !autoboot->filename.empty() && !FileSystem::FileExists(autoboot->filename.c_str()))
  {
    QMessageBox::critical(
      nullptr, qApp->translate("QtHost", "Error"),
      qApp->translate("QtHost", "File '%1' does not exist.").arg(QString::fromStdString(autoboot->filename)));
    return false;
  }

  if (state_index.has_value())
  {
    AutoBoot(autoboot);

    if (autoboot->filename.empty())
    {
      // loading global state, -1 means resume the last game
      if (state_index.value() < 0)
        autoboot->save_state = System::GetMostRecentResumeSaveStatePath();
      else
        autoboot->save_state = System::GetGlobalSaveStateFileName(state_index.value());
    }
    else
    {
      // loading game state
      const std::string game_serial(GameDatabase::GetSerialForPath(autoboot->filename.c_str()));
      autoboot->save_state = System::GetGameSaveStateFileName(game_serial, state_index.value());
    }

    if (autoboot->save_state.empty() || !FileSystem::FileExists(autoboot->save_state.c_str()))
    {
      QMessageBox::critical(nullptr, qApp->translate("QtHost", "Error"),
                            qApp->translate("QtHost", "The specified save state does not exist."));
      return false;
    }
  }

  // check autoboot parameters, if we set something like fullscreen without a bios
  // or disc, we don't want to actually start.
  if (autoboot && autoboot->filename.empty() && autoboot->save_state.empty() && !starting_bios)
    autoboot.reset();

  // if we don't have autoboot, we definitely don't want batch mode (because that'll skip
  // scanning the game list).
  if (s_batch_mode && !autoboot && !s_start_fullscreen_ui)
  {
    QMessageBox::critical(
      nullptr, qApp->translate("QtHost", "Error"),
      s_nogui_mode ? qApp->translate("QtHost", "Cannot use no-gui mode, because no boot filename was specified.") :
                     qApp->translate("QtHost", "Cannot use batch mode, because no boot filename was specified."));
    return false;
  }

  return true;
}

int main(int argc, char* argv[])
{
  CrashHandler::Install();

  QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
  QtHost::RegisterTypes();

  QApplication app(argc, argv);

  std::shared_ptr<SystemBootParameters> autoboot;
  if (!QtHost::ParseCommandLineParametersAndInitializeConfig(app, autoboot))
    return EXIT_FAILURE;

  // Set theme before creating any windows.
  MainWindow::updateApplicationTheme();

  // Start up the CPU thread.
  MainWindow* main_window = new MainWindow();
  QtHost::HookSignals();
  EmuThread::start();

  // Create all window objects, the emuthread might still be starting up at this point.
  main_window->initialize();

  // When running in batch mode, ensure game list is loaded, but don't scan for any new files.
  if (!s_batch_mode)
    main_window->refreshGameList(false);
  else
    GameList::Refresh(false, true);

  // Don't bother showing the window in no-gui mode.
  if (!s_nogui_mode)
    main_window->show();

  // Initialize big picture mode if requested.
  if (s_start_fullscreen_ui)
    g_emu_thread->startFullscreenUI();
  else
    s_start_fullscreen_ui_fullscreen = false;

  // Skip the update check if we're booting a game directly.
  if (autoboot)
    g_emu_thread->bootSystem(std::move(autoboot));
  else if (!s_nogui_mode)
    main_window->startupUpdateCheck();

  // This doesn't return until we exit.
  const int result = app.exec();

  // Shutting down.
  EmuThread::stop();

  // Close main window.
  if (g_main_window)
  {
    g_main_window->close();
    delete g_main_window;
    Assert(!g_main_window);
  }

  // Ensure settings are saved.
  if (s_settings_save_timer)
  {
    s_settings_save_timer.reset();
    QtHost::SaveSettings();
  }

  // Ensure log is flushed.
  Log::SetFileOutputParams(false, nullptr);

  return result;
}
