#pragma once

#include <esp32-hal-log.h>
#define DCC_LOG_LVL  ARDUHAL_LOG_LEVEL_INFO

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


#if DCC_LOG_LVL >= ARDUHAL_LOG_LEVEL_NONE
#include <etl/span.h>

//!!! NOT REENTRANT!
inline char* fmt_span(const etl::span<const uint8_t> bytes) {
    static char buf[100];
    size_t n = 0;
    n += snprintf(buf, sizeof(buf), "[len=%d:", bytes.size());
    for(size_t i=0; i<bytes.size() && n < sizeof(buf); i++) {
        n += snprintf(buf+n, sizeof(buf)-n, "%02X ", bytes[i]);
    }
    if(bytes.size()>0) n--;
    n += snprintf(buf+n, sizeof(buf)-n, "]");
    return buf;
}
#else
#define fmt_span(...)  ""
#endif
