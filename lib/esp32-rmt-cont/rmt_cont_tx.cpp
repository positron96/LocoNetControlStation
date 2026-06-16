/**
 * @file rmt_cont_tx.cpp
 * @brief Continuous (gapless) RMT TX — implementation
 */

#include "rmt_cont_tx.h"

#include <cstring>
#include "soc/rmt_struct.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_periph.h"
#include "hal/rmt_ll.h"
#include "hal/gpio_hal.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

static const char *TAG = "rmt_cont_tx";

// ─── Internal state ──────────────────────────────────────────────────────────

static constexpr uint32_t RMT_APB_SOURCE_HZ = 80 * 1000 * 1000; // ESP32 APB clock

typedef struct {
    bool active;
    uint8_t channel;
    uint16_t total_items;
    uint16_t half_items;
    rmt_cont_tx_fill_cb_t fill_cb;
    void *ctx;
    intr_handle_t intr_handle;
} rmt_cont_tx_runtime_t;

static rmt_cont_tx_runtime_t s_channels[SOC_RMT_CHANNELS_PER_GROUP];

static volatile rmt_symbol_word_t *channel_mem(uint8_t channel) {
    return reinterpret_cast<volatile rmt_symbol_word_t *>(
        RMT_CHANNEL_MEM(channel));
}

// ─── ISR ─────────────────────────────────────────────────────────────────────

static void IRAM_ATTR rmt_cont_isr(void *arg) {
    rmt_cont_tx_runtime_t *ch = reinterpret_cast<rmt_cont_tx_runtime_t *>(arg);
    if (!ch || !ch->active || !ch->fill_cb) {
        return;
    }

    rmt_dev_t *dev = &RMT;
    uint32_t status = dev->int_st.val;
    uint8_t channel = ch->channel;

    // TX threshold (halfway) → HW is reading second half, refill first half
    if (status & RMT_LL_EVENT_TX_THRES(channel)) {
        rmt_ll_clear_interrupt_status(dev, RMT_LL_EVENT_TX_THRES(channel));
        ch->fill_cb(channel_mem(channel), ch->half_items, ch->ctx);
    }

    // TX done (full cycle in conti mode) → HW wrapped to start, refill second half
    if (status & RMT_LL_EVENT_TX_DONE(channel)) {
        rmt_ll_clear_interrupt_status(dev, RMT_LL_EVENT_TX_DONE(channel));
        ch->fill_cb(channel_mem(channel) + ch->half_items, ch->half_items, ch->ctx);
    }

    // Error
    if (status & RMT_LL_EVENT_TX_ERROR(channel)) {
        rmt_ll_clear_interrupt_status(dev, RMT_LL_EVENT_TX_ERROR(channel));
        ESP_EARLY_LOGW(TAG, "TX error on ch %d", channel);
    }
}

// ─── Public API ──────────────────────────────────────────────────────────────

esp_err_t rmt_cont_tx_start(const rmt_cont_tx_config_t *config) {
    if (!config || !config->fill_cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->resolution_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->channel >= SOC_RMT_CHANNELS_PER_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->mem_blocks == 0 ||
        config->channel + config->mem_blocks > SOC_RMT_CHANNELS_PER_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }

    rmt_cont_tx_runtime_t *ch = &s_channels[config->channel];
    if (ch->active) {
        return ESP_ERR_INVALID_STATE;
    }

    ch->active = true;
    ch->channel = config->channel;
    ch->fill_cb = config->fill_cb;
    ch->ctx = config->ctx;
    ch->total_items = config->mem_blocks * RMT_CONT_ITEMS_PER_BLOCK;
    ch->half_items = ch->total_items / 2;
    ch->intr_handle = nullptr;

    uint8_t channel = ch->channel;

    rmt_dev_t *dev = &RMT;

    // Peripheral clock & memory
    rmt_ll_enable_periph_clock(dev, true);
    rmt_ll_power_down_mem(dev, false);
    rmt_ll_enable_mem_access_nonfifo(dev, true);

    // GPIO
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[config->gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(config->gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(config->gpio_num,
        rmt_periph_signals.groups[0].channels[channel].tx_sig, false, false);

    // Channel clock
    const uint32_t source_hz = RMT_APB_SOURCE_HZ;
    uint32_t real_div = (source_hz + config->resolution_hz / 2) / config->resolution_hz;
    if (real_div < 1 || real_div > 256) {
        ESP_LOGE(TAG, "resolution_hz out of range: %lu (div=%lu)",
                 (unsigned long)config->resolution_hz, (unsigned long)real_div);
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t real_resolution_hz = source_hz / real_div;

    rmt_ll_set_group_clock_src(dev, channel, RMT_CLK_SRC_APB, 0, 0, 0);
    rmt_ll_tx_set_channel_clock_div(dev, channel, real_div);

    // Memory blocks
    rmt_ll_tx_set_mem_blocks(dev, channel, config->mem_blocks);

    // Continuous loop + wrap
    rmt_ll_tx_enable_loop(dev, channel, true);
    rmt_ll_tx_enable_wrap(dev, channel, true);

    // Idle level
    rmt_ll_tx_fix_idle_level(dev, channel, config->idle_level, true);

    // Threshold at halfway
    rmt_ll_tx_set_limit(dev, channel, ch->half_items);

    // Pre-fill entire buffer
    volatile rmt_symbol_word_t *mem = channel_mem(channel);
    ch->fill_cb(mem, ch->half_items, ch->ctx);
    ch->fill_cb(mem + ch->half_items, ch->half_items, ch->ctx);

    // Register ISR (shared source, channel context passed via arg)
    esp_err_t err = esp_intr_alloc(ETS_RMT_INTR_SOURCE,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_SHARED,
        rmt_cont_isr, ch, &ch->intr_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate ISR: %d", err);
        ch->active = false;
        ch->fill_cb = nullptr;
        ch->ctx = nullptr;
        return err;
    }

    // Enable TX_DONE + TX_THRES interrupts
    uint32_t int_mask = RMT_LL_EVENT_TX_DONE(channel)
                      | RMT_LL_EVENT_TX_THRES(channel);
    rmt_ll_enable_interrupt(dev, int_mask, true);
    rmt_ll_clear_interrupt_status(dev, int_mask);

    // Start
    rmt_ll_tx_reset_pointer(dev, channel);
    rmt_ll_tx_start(dev, channel);

    if (real_resolution_hz != config->resolution_hz) {
        ESP_LOGW(TAG, "resolution loss: requested=%lu real=%lu",
                 (unsigned long)config->resolution_hz, (unsigned long)real_resolution_hz);
    }
    ESP_LOGI(TAG, "Started on GPIO %d, ch %d, %d items (%d per half)",
             config->gpio_num, channel, ch->total_items, ch->half_items);
    return ESP_OK;
}

esp_err_t rmt_cont_tx_stop(uint8_t channel) {
    if (channel >= SOC_RMT_CHANNELS_PER_GROUP) {
        return ESP_ERR_INVALID_ARG;
    }

    rmt_cont_tx_runtime_t *ch = &s_channels[channel];
    if (!ch->active) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_dev_t *dev = &RMT;

    // Disable interrupts
    uint32_t int_mask = RMT_LL_EVENT_TX_DONE(channel)
                      | RMT_LL_EVENT_TX_THRES(channel)
                      | RMT_LL_EVENT_TX_ERROR(channel);
    rmt_ll_enable_interrupt(dev, int_mask, false);

    // Stop looping (finishes current cycle then stops)
    rmt_ll_tx_enable_loop(dev, channel, false);

    // Free ISR
    if (ch->intr_handle) {
        esp_intr_free(ch->intr_handle);
        ch->intr_handle = nullptr;
    }

    ch->active = false;
    ch->fill_cb = nullptr;
    ch->ctx = nullptr;
    ch->total_items = 0;
    ch->half_items = 0;

    ESP_LOGI(TAG, "Stopped ch %d", channel);
    return ESP_OK;
}
