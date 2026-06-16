/**
 * @file rmt_cont_tx.h
 * @brief Continuous (gapless) RMT TX streaming on ESP32 using LL API
 *
 * Uses a ping-pong double-buffer approach:
 *  - Channel uses N memory blocks (each block = 64 RMT items)
 *  - tx_conti_mode auto-restarts from the beginning of memory
 *  - TX_THRES interrupt fires at the halfway point → user refills first half
 *  - TX_DONE interrupt fires at the end → user refills second half
 *
 * Assumes exclusive ownership of the RMT peripheral and its interrupt.
 */

#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "soc/rmt_struct.h"
#include "hal/rmt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RMT_CONT_ITEMS_PER_BLOCK 64

/**
 * @brief Callback invoked from ISR to fill RMT items into hardware memory.
 *
 * @param items   Pointer into channel's hardware RMT memory at the region to fill
 * @param count   Number of rmt_symbol_word_t items to write
 * @param ctx     User context pointer from rmt_cont_tx_config_t::ctx
 *
 * Must be placed in IRAM. Must complete before the other half is consumed.
 */
typedef void (*rmt_cont_tx_fill_cb_t)(volatile rmt_symbol_word_t *items,
                                      uint16_t count, void *ctx);

/**
 * @brief Configuration for continuous RMT TX.
 */
typedef struct {
    gpio_num_t gpio_num;       /**< Output GPIO pin */
    uint8_t channel;           /**< RMT TX channel number (0-7) */
    uint8_t mem_blocks;        /**< Number of 64-item memory blocks (1-8, channel+mem_blocks <= 8) */
    uint32_t resolution_hz;    /**< Requested channel tick resolution in Hz (derived from APB clock) */
    uint8_t idle_level;        /**< Output level when idle (0 or 1) */
    rmt_cont_tx_fill_cb_t fill_cb;  /**< ISR callback to fill RMT items (IRAM) */
    void *ctx;                 /**< User context passed to fill_cb */
} rmt_cont_tx_config_t;

/**
 * @brief Start continuous RMT TX.
 *
 * Configures the RMT peripheral, registers the ISR, pre-fills the buffer
 * via fill_cb, and starts transmission.
 *
 * @param config  Configuration parameters
 * @return ESP_OK on success, or an error code
 */
esp_err_t rmt_cont_tx_start(const rmt_cont_tx_config_t *config);

/**
 * @brief Stop continuous RMT TX and release the ISR.
 *
 * @param channel  RMT TX channel number to stop
 * @return ESP_OK on success, or an error code
 */
esp_err_t rmt_cont_tx_stop(uint8_t channel);

#ifdef __cplusplus
}
#endif
