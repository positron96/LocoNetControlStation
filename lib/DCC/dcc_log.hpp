#pragma once

#define DCC_LOG_LVL  ARDUHAL_LOG_LEVEL_DEBUG

#include  "rom/ets_sys.h"  // for ets_printf

#if DCC_LOG_LVL >= ARDUHAL_LOG_LEVEL_WARN
    #define DCC_LOGW(format, ...) log_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
    #define DCC_LOGW_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(W, format), ##__VA_ARGS__)
#else
    #define DCC_LOGW(...)
    #define DCC_LOGW_ISR(...)
#endif
#if DCC_LOG_LVL >= ARDUHAL_LOG_LEVEL_INFO
    #define DCC_LOGI(format, ...) log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__)
#else
    #define DCC_LOGI(...)
#endif
#if DCC_LOG_LVL >= ARDUHAL_LOG_LEVEL_DEBUG
    #define DCC_LOGD(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
    #define DCC_LOGD_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#else
    #define DCC_LOGD(...)
    #define DCC_LOGD_ISR(...)
#endif
