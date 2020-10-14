#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "types.h"

class StateWrapper;

// Event callback type. Second parameter is the number of cycles the event was executed "late".
using TimingEventCallback = std::function<void(TickCount ticks, TickCount ticks_late)>;

class TimingEvent
{
public:
  TimingEvent(std::string name, TickCount period, TickCount interval, TimingEventCallback callback);
  ~TimingEvent();

  const std::string& GetName() const { return m_name; }
  bool IsActive() const { return m_active; }

  // Returns the number of ticks between each event.
  ALWAYS_INLINE TickCount GetPeriod() const { return m_period; }
  ALWAYS_INLINE TickCount GetInterval() const { return m_interval; }

  // Includes pending time.
  TickCount GetTicksSinceLastExecution() const;
  TickCount GetTicksUntilNextExecution() const;

  void Schedule(TickCount ticks);
  void SetIntervalAndSchedule(TickCount ticks);
  void SetPeriodAndSchedule(TickCount ticks);

  void Reset();

  // Services the event with the current accmulated time. If force is set, when not enough time is pending to
  // simulate a single cycle, the callback will still be invoked, otherwise it won't be.
  void InvokeEarly(bool force = false);

  // Deactivates the event, preventing it from firing again.
  // Do not call within a callback, return Deactivate instead.
  void Activate();
  void Deactivate();

  ALWAYS_INLINE void SetState(bool active)
  {
    if (active)
      Activate();
    else
      Deactivate();
  }

  // Directly alters the interval of the event.
  void SetInterval(TickCount interval) { m_interval = interval; }
  void SetPeriod(TickCount period) { m_period = period; }

  TimingEvent* prev = nullptr;
  TimingEvent* next = nullptr;

  u64 m_next_run_time;
  u64 m_last_run_time;
  TickCount m_period;
  TickCount m_interval;

  TimingEventCallback m_callback;
  std::string m_name;
  bool m_active;
};

namespace TimingEvents {

u64 GetGlobalTickCounter();

void Initialize();
void Reset();
void Shutdown();

/// Creates a new event.
std::unique_ptr<TimingEvent> CreateTimingEvent(std::string name, TickCount period, TickCount interval,
                                               TimingEventCallback callback, bool activate);

/// Serialization.
bool DoState(StateWrapper& sw);

void RunEvents();

void UpdateCPUDowncount();



} // namespace TimingEventManager