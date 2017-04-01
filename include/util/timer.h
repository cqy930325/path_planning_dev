/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_TIMER_H_
#define UTIL_INCLUDE_UTIL_TIMER_H_

#include <chrono>

#include <util/debug.h>

namespace PI {
namespace util {

/**
 * A non-reusable timer API.
 */
class Timer {
 public:
  Timer();
  ~Timer();

 protected:
  /**
   * @brief Start timing.
   * @warning Timer can only be started once.
   */
  void Start();

  /**
   * @brief Stop timing.
   * @warning Timer must be started. Timer can only be stopped once.
   */
  void Stop();

  /**
   * @return The starting time of the timer.
   * @warning The timer must be started.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>&
      start_time() const;

  /**
   * @return The ending time of the timer.
   * @warning The timer must be stopped.
   */
  const std::chrono::time_point<std::chrono::high_resolution_clock>&
      end_time() const;

 private:
  /// A flag to specify whether the timer has been started or not.
  bool has_started_;
  /// Start time.
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  /// A flag to specify whether the timer has been stopped or not.
  bool has_ended_;
  /// Stopped time.
  std::chrono::time_point<std::chrono::high_resolution_clock> end_;
};

/**
 * A timer outputting time duration in microsecond.
 */
class MicrosecondTimer : public Timer {
 public:
  MicrosecondTimer();
  ~MicrosecondTimer();

  /**
   * @brief Start the timer.
   */
  void Start();

  /**
   * @brief Stop the timer.
   * @return Microsecond between the starting and stopping time.
   */
  int Stop();
};

/**
 * A microsecond timer that will stop while it goes out of scope.
 * Result will be printed to VLOG at a specified level.
 */
class ScopedMicrosecondTimer : public MicrosecondTimer {
 public:
  /**
   * @brief Constructor.
   * @param time_id A string to distinguish the timer in the log.
   * @param vlog_level At which level to log the duration of the timer. If
   *                   negative, use LOG(INFO).
   */
  explicit ScopedMicrosecondTimer(const std::string &timer_id,
                                  const int vlog_level = -1);
  ~ScopedMicrosecondTimer();

 private:
  /// A string to distinguish the timer in the log.
  const std::string id_;
  /// At which level to log the duration of the timer. If negative, use
  /// LOG(INFO).
  const int vlog_level_;
};

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_TIMER_H_
