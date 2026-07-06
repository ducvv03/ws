#pragma once

// Log severity levels. The sink receives the level so a real backend can
// filter (e.g. drop Debug in release).
typedef enum HsmLogLevel {
    HSM_LOG_DEBUG,
    HSM_LOG_INFO,
    HSM_LOG_WARN,
    HSM_LOG_ERROR
} HsmLogLevel;

const char* hsm_log_level_name(HsmLogLevel level);

// Pluggable, printf-style logging module.
//
// Usage:
//   hsm_log_info("enter %s id=%d", state->name, state->id);
//   hsm_log_error("unknown state id=%d", code);
//   hsm_log(HSM_LOG_WARN, "low hp: %d", hp);   // dạng tổng quát
//
// Port to your own backend (sink receives level + already-formatted line):
//   hsm_log_set_sink(my_sink);
//
// Silence everything:
//   hsm_log_set_sink(NULL);
typedef void (*HsmLogSink)(HsmLogLevel level, const char* line);

// Install a custom sink (or NULL to disable logging). Default sink prints
// to stdout.
void hsm_log_set_sink(HsmLogSink sink);

// Generic + per-level printf-style entry points.
void hsm_log(HsmLogLevel level, const char* fmt, ...);
void hsm_log_debug(const char* fmt, ...);
void hsm_log_info(const char* fmt, ...);
void hsm_log_warn(const char* fmt, ...);
void hsm_log_error(const char* fmt, ...);
