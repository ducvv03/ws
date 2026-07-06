#pragma once
#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <cstdarg>
#include <cstdio>

namespace hsm {

// Log severity levels. The sink receives the level so a real backend can
// filter (e.g. drop Debug in release).
enum class Level { Debug, Info, Warn, Error };

inline const char* levelName(Level lvl) {
    switch (lvl) {
        case Level::Debug: return "DEBUG";
        case Level::Info:  return "INFO";
        case Level::Warn:  return "WARN";
        case Level::Error: return "ERROR";
    }
    return "?";
}

// Pluggable, printf-style logging module.
//
// Usage:
//   Logger::info("enter %s id=%d", state.name().c_str(), state.id());
//   Logger::error("unknown state id=%d", code);
//
// Port to your own backend (sink receives level + already-formatted line):
//   Logger::set([](Level lvl, const std::string& line) {
//       MyGameLog::write(levelName(lvl), line);
//   });
//
// Silence everything:
//   Logger::set(nullptr);
//
// NOTE (printf contract): pass C strings via %s with .c_str(), not std::string
// objects — variadic args are not type-checked.
class Logger {
public:
    typedef std::function<void(Level, const std::string&)> Sink;

    static Sink& sink() {
        static Sink s = [](Level lvl, const std::string& line) {
            std::cout << "[hsm][" << levelName(lvl) << "] " << line << "\n";
        };
        return s;
    }

    // Install a custom sink (or nullptr to disable logging).
    static void set(Sink fn) { sink() = fn; }

    // Generic + per-level printf-style entry points.
    static void log(Level lvl, const char* fmt, ...) {
        va_list args; va_start(args, fmt);
        emit(lvl, fmt, args);
        va_end(args);
    }
    static void debug(const char* fmt, ...) {
        va_list args; va_start(args, fmt);
        emit(Level::Debug, fmt, args);
        va_end(args);
    }
    static void info(const char* fmt, ...) {
        va_list args; va_start(args, fmt);
        emit(Level::Info, fmt, args);
        va_end(args);
    }
    static void warn(const char* fmt, ...) {
        va_list args; va_start(args, fmt);
        emit(Level::Warn, fmt, args);
        va_end(args);
    }
    static void error(const char* fmt, ...) {
        va_list args; va_start(args, fmt);
        emit(Level::Error, fmt, args);
        va_end(args);
    }

private:
    static void emit(Level lvl, const char* fmt, va_list args) {
        Sink& s = sink();
        if (!s) return;
        s(lvl, format(fmt, args));
    }

    static std::string format(const char* fmt, va_list args) {
        va_list copy;
        va_copy(copy, args);
        int n = std::vsnprintf(nullptr, 0, fmt, copy);   // measure
        va_end(copy);
        if (n <= 0) return std::string();
        std::vector<char> buf(static_cast<size_t>(n) + 1);
        std::vsnprintf(buf.data(), buf.size(), fmt, args); // format
        return std::string(buf.data(), static_cast<size_t>(n));
    }
};

} // namespace hsm
