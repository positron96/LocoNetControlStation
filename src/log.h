/**
 * Respects variables GLOBAL_LOG_LEVEL, LOG_LEVEL, FILE_LOG_LEVEL
 *   (in this priority, FILE_LOG_LEVEL has the highest)
 * Allows inclusion several times, overwrites its defines each time.
 *
 * Usage:
 *
 * // in Makefile or other build system: GLOBAL_LOG_LEVEL = LEVEL_INFO
 * #define LOG_LEVEL LEVEL_DEBUG
 * #include "log.h"
 */

#ifndef LOG_H_

    // if it's a first time include, define all the constants

    #define LOG_H_

    #include <esp32-hal-log.h>

    #define LEVEL_NONE     (0)
    #define LEVEL_ERROR    (1)
    #define LEVEL_WARN     (2)
    #define LEVEL_INFO     (3)
    #define LEVEL_DEBUG    (4)
    #define LEVEL_VERBOSE  (5)

#else
    // It was already included, undefine all logging macros
    //  and redefine them with new ifdefs.
    #undef LOGV
    #undef LOGV_ISR
    #undef LOGD
    #undef LOGD_ISR
    #undef LOGI
    #undef LOGI_ISR
    #undef LOGW
    #undef LOGE
    #undef LOGE_ISR
#endif

// LOG_LEVEL takes priority over GLOBAL_LOG_LEVEL
#if defined(GLOBAL_LOG_LEVEL) && !defined(LOG_LEVEL)
    #define LOG_LEVEL  GLOBAL_LOG_LEVEL
#endif

// if FILE_LOG_LEVEL is set, it takes priority
#if defined(FILE_LOG_LEVEL)
#if defined(LOG_LEVEL)
  #undef LOG_LEVEL
#endif
#define LOG_LEVEL  FILE_LOG_LEVEL
#endif


#if LOG_LEVEL >= LEVEL_VERBOSE
#define LOGV(format, ...) log_printf(ARDUHAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#define LOGV_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(V, format), ##__VA_ARGS__)
#else
#define LOGV(format, ...)
#define LOGV_ISR(format, ...)
#endif

#if LOG_LEVEL >= LEVEL_DEBUG
#define LOGD(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#define LOGD_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#else
#define LOGD(format, ...)
#define LOGD_ISR(format, ...)
#endif

#if LOG_LEVEL >= LEVEL_INFO
#define LOGI(format, ...) log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#define LOGI_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#else
#define LOGI(format, ...)
#define LOGI_ISR(format, ...)
#endif

#if LOG_LEVEL >= LEVEL_WARN
#define LOGW(format, ...) log_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
#define LOGW_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
#else
#define LOGW(format, ...)
#define LOGW_ISR(format, ...)
#endif

#if LOG_LEVEL >= LEVEL_ERROR
#define LOGE(format, ...) log_printf(ARDUHAL_LOG_FORMAT(E, format), ##__VA_ARGS__)
#define LOGE_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(E, format), ##__VA_ARGS__)
#else
#define LOGE(format, ...)
#define LOGE_ISR(format, ...)
#endif
