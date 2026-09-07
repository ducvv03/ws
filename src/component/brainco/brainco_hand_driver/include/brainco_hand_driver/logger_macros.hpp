#pragma once

#include <chrono>
#include <cstdarg>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace brainco_hand_driver
{
namespace logging
{

inline std::string get_basename(const std::string & path)
{
  return std::filesystem::path(path).filename().string();
}

inline std::string format_location(const char * file, int line, const char * function)
{
  return get_basename(file) + ":" + std::to_string(line) + " " + std::string(function);
}

inline std::string format_string(const char * format, ...)
{
  char buffer[1024];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return std::string(buffer);
}

// Rate limiter for logs on the ros2_control read()/write() path.
//
// The hand runs at the controller_manager update_rate (100 Hz in the shipped
// configs). A level-triggered log on a failure that persists — a hand that
// stopped answering, say — would emit 100 lines per second, each flushed by the
// std::endl in the macros below, which disturbs the very loop being diagnosed.
//
// Not thread-safe by design: the call sites are read()/write(), driven by the
// single controller-manager thread.
inline bool should_emit(std::chrono::steady_clock::time_point & last_emit, int64_t interval_ms)
{
  const auto now = std::chrono::steady_clock::now();
  const bool never_emitted = last_emit.time_since_epoch().count() == 0;
  if (!never_emitted && (now - last_emit) < std::chrono::milliseconds(interval_ms))
  {
    return false;
  }
  last_emit = now;
  return true;
}

inline std::string now_string()
{
  auto now = std::chrono::system_clock::now();
  auto now_e8 = now + std::chrono::hours(8);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_e8.time_since_epoch()) % 1000;
  auto time_t = std::chrono::system_clock::to_time_t(now_e8);
  auto tm = *std::gmtime(&time_t);
  std::ostringstream oss_time;
  oss_time << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3)
           << ms.count();
  return oss_time.str();
}

}  // namespace logging
}  // namespace brainco_hand_driver

#define BRAINCO_HAND_LOG_DEBUG(format, ...)                                                        \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " DEBUG] ["                \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_INFO(format, ...)                                                         \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " INFO] ["                 \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_WARN(format, ...)                                                         \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " WARN] ["                 \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

#define BRAINCO_HAND_LOG_ERROR(format, ...)                                                        \
  do                                                                                               \
  {                                                                                                \
    std::cout << "[" << ::brainco_hand_driver::logging::now_string() << " ERROR] ["                \
              << ::brainco_hand_driver::logging::format_location(__FILE__, __LINE__, __FUNCTION__) \
              << "] " << ::brainco_hand_driver::logging::format_string(format, ##__VA_ARGS__)      \
              << std::endl;                                                                        \
  } while (0)

// Throttled variants for read()/write(). Each expansion owns its own timer, so
// two call sites never share a budget. Suffix the message with the interval so a
// reader knows the line is rate-limited rather than the fault being intermittent.
#define BRAINCO_HAND_LOG_THROTTLE_IMPL(level_macro, interval_ms, format, ...)                      \
  do                                                                                               \
  {                                                                                                \
    static std::chrono::steady_clock::time_point brainco_hand_last_emit{};                         \
    if (::brainco_hand_driver::logging::should_emit(brainco_hand_last_emit, (interval_ms)))         \
    {                                                                                              \
      level_macro(format, ##__VA_ARGS__);                                                          \
    }                                                                                              \
  } while (0)

#define BRAINCO_HAND_LOG_DEBUG_THROTTLE(interval_ms, format, ...)                                  \
  BRAINCO_HAND_LOG_THROTTLE_IMPL(BRAINCO_HAND_LOG_DEBUG, interval_ms, format, ##__VA_ARGS__)

#define BRAINCO_HAND_LOG_INFO_THROTTLE(interval_ms, format, ...)                                   \
  BRAINCO_HAND_LOG_THROTTLE_IMPL(BRAINCO_HAND_LOG_INFO, interval_ms, format, ##__VA_ARGS__)

#define BRAINCO_HAND_LOG_WARN_THROTTLE(interval_ms, format, ...)                                   \
  BRAINCO_HAND_LOG_THROTTLE_IMPL(BRAINCO_HAND_LOG_WARN, interval_ms, format, ##__VA_ARGS__)

#define BRAINCO_HAND_LOG_ERROR_THROTTLE(interval_ms, format, ...)                                  \
  BRAINCO_HAND_LOG_THROTTLE_IMPL(BRAINCO_HAND_LOG_ERROR, interval_ms, format, ##__VA_ARGS__)
