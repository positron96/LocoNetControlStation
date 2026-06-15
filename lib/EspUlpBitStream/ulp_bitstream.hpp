/**
 * ESP32 ULP FSM coprocessor: circular-buffered bit streaming
 * on 2 GPIOs every N µs.
 *
 * The ULP maintains a read position; the main core maintains a
 * write position.  Each ULP wake outputs one bit from buf_a[] and
 * buf_b[] at read_pos, then advances read_pos.  If read_pos ==
 * write_pos the buffer is empty and the ULP halts until the main
 * core writes more data and advances write_pos.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "driver/gpio.h"

namespace ulp_bitstream {

/* ---------- Runtime configuration passed to init ---------- */
struct config_t {
    gpio_num_t gpio_a;      /* must be RTC-capable (output-capable) GPIO */
    gpio_num_t gpio_b;      /* must be RTC-capable (output-capable) GPIO */
    uint16_t   buf_len;     /* circular buffer length (usable slots = buf_len - 1) */
    uint32_t   timer_us;    /* ULP wake period in microseconds */
};

/* ---------- API ---------- */

/**
 * Initialize ULP bitstream (does NOT start it).
 * Configures GPIOs, builds ULP program, loads into RTC slow memory.
 * Returns 0 on success, -1 if GPIOs are not RTC-capable or buffer too large.
 */
int init(const config_t &cfg);

/** Start ULP timer — begins continuous output. */
void start(void);

/** Stop ULP timer — halts output. Buffers and program remain loaded. */
void stop(void);

/** Current ULP read position (0 .. buf_len-1). Updated within ULP. */
volatile uint16_t get_read_pos(void);

/** Current ULP write position (0 .. buf_len-1). */
uint16_t get_write_pos(void);

/** Configured buffer length. */
uint16_t buf_len(void);

/**
 * Number of free slots the main core may write before the buffer is full.
 * Always leaves one slot empty to distinguish full from empty.
 */
volatile size_t available(void);

/**
 * Append bits into the circular buffer at the current write position.
 * Caller must ensure len <= ulp_bitstream::available().
 * @param len       number of bits to write
 * @param bits_a    len bytes, each 0 or 1
 * @param bits_b    len bytes, each 0 or 1
 */
void write(size_t len,
    const uint8_t *bits_a,
    const uint8_t *bits_b);

}
