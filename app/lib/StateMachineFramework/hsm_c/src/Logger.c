#include "hsm/Logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

static void default_sink(HsmLogLevel level, const char* line) {
    printf("[hsm][%s] %s\n", hsm_log_level_name(level), line);
}

static HsmLogSink g_sink = default_sink;

const char* hsm_log_level_name(HsmLogLevel level) {
    switch (level) {
        case HSM_LOG_DEBUG: return "DEBUG";
        case HSM_LOG_INFO:  return "INFO";
        case HSM_LOG_WARN:  return "WARN";
        case HSM_LOG_ERROR: return "ERROR";
    }
    return "?";
}

void hsm_log_set_sink(HsmLogSink sink) {
    g_sink = sink;
}

static void emit(HsmLogLevel level, const char* fmt, va_list args) {
    if (!g_sink) return;

    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(NULL, 0, fmt, copy);
    va_end(copy);
    if (n < 0) return;

    char* buf = (char*)malloc((size_t)n + 1);
    if (!buf) return;
    vsnprintf(buf, (size_t)n + 1, fmt, args);

    g_sink(level, buf);
    free(buf);
}

void hsm_log(HsmLogLevel level, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    emit(level, fmt, args);
    va_end(args);
}

void hsm_log_debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    emit(HSM_LOG_DEBUG, fmt, args);
    va_end(args);
}

void hsm_log_info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    emit(HSM_LOG_INFO, fmt, args);
    va_end(args);
}

void hsm_log_warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    emit(HSM_LOG_WARN, fmt, args);
    va_end(args);
}

void hsm_log_error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    emit(HSM_LOG_ERROR, fmt, args);
    va_end(args);
}
