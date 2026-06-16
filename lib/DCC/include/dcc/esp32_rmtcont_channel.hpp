#pragma once

#include "esp32_channel.hpp"

#include <esp_attr.h>
#include <rmt_cont_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

namespace dcc {

/**
 * A DCC channel that outputs DCC waveform using continuous ESP32 RMT LL API.
 *
 * The ISR callback posts a fill-request (pointer + count) to a high-priority
 * FreeRTOS task via a queue.  Packet encoding happens in the task context, not
 * in the ISR, so the ISR returns in O(1) and all normal FreeRTOS APIs are safe.
 */
class ESP32RMTChannel : public ESP32Channel {
public:
    ESP32RMTChannel(
        uint8_t outputPin,
        uint8_t enPin,
        uint8_t sensePin,
        BasePacketList &packets
    ) : ESP32Channel{outputPin, enPin, sensePin, packets}
    { }

    void begin() override {
        ESP32Channel::begin();

        if (_channel >= 8) {
            DCC_LOGW("No free RMT channel for GPIO %d", _outputPin);
            return;
        }

        // Create queue before starting TX so the pre-fill call from rmt_cont_tx_start can post
        _fillQueue = xQueueCreate(1, sizeof(FillRequest));
        if (!_fillQueue) {
            DCC_LOGW("Failed to create fill queue for ch %d", _channel);
            return;
        }

        rmt_cont_tx_config_t cfg{};
        cfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        cfg.channel = _channel;
        cfg.mem_blocks    = 1;
        cfg.resolution_hz = 1'000'000; // 1 tick = 1 µs
        cfg.idle_level    = 0;
        cfg.fill_cb       = &fillCallback;
        cfg.ctx           = this;

        _running = true;

        if (rmt_cont_tx_start(&cfg) != ESP_OK) {
            DCC_LOGW("RMT continuous TX start failed on GPIO %d (ch %d)", _outputPin, _channel);
            _running = false;
            vQueueDelete(_fillQueue);
            _fillQueue = nullptr;
            return;
        }

        // High priority so it preempts lower-priority work and finishes before
        // RMT consumes the half-buffer.
        if (xTaskCreate(fillTask_c, "dcc_rmt_fill", 2048, this,
                        configMAX_PRIORITIES - 1, &_fillTask) != pdPASS) {
            DCC_LOGW("Failed to create fill task for ch %d", _channel);
            _running = false;
            rmt_cont_tx_stop(_channel);
            vQueueDelete(_fillQueue);
            _fillQueue = nullptr;
        }
    }

    void end() override {
        _running = false;
        if (_fillTask) {
            if (_fillQueue) {
                FillRequest dummy{nullptr, 0};
                xQueueSend(_fillQueue, &dummy, pdMS_TO_TICKS(10));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            _fillTask = nullptr;
        }
        if (_fillQueue) {
            vQueueDelete(_fillQueue);
            _fillQueue = nullptr;
        }
        if (_channel < 8) {
            rmt_cont_tx_stop(_channel);
        }
        ESP32Channel::end();
    }

private:
    struct FillRequest {
        volatile rmt_symbol_word_t *items;
        uint16_t count;
    };

    QueueHandle_t _fillQueue{nullptr}; // ISR → task
    TaskHandle_t  _fillTask{nullptr};

    uint8_t _channel{s_nextChannel++};

    static constexpr uint16_t DCC_ONE_HALF_US  = 58;
    static constexpr uint16_t DCC_ZERO_HALF_US = 116;
    inline static uint8_t s_nextChannel{0};
    volatile bool _running{false};

    PacketBitsWithRepeats _activePacket{};
    size_t _bitPos{0};

    // Runs in task context — may call any FreeRTOS/packet API freely.
    void fillSymbols(volatile rmt_symbol_word_t *items, uint16_t count) {
        for (uint16_t i = 0; i < count; ++i) {
            if (_bitPos >= _activePacket.packet.size_bits) {
                _bitPos = 0;
                if (_activePacket.nRepeats > 0) {
                    --_activePacket.nRepeats;
                } else {
                    PacketWithRepeats bytes;
                    if (packets.fetch_next_packet(bytes)) {
                        _activePacket = PacketBitsWithRepeats::from_packet(bytes, DEF_PREAMBLE_LEN);
                    } else {
                        _activePacket = {idle_packet_bits, 1};
                    }
                }
            }

            const bool isOne = _activePacket.packet.bit_at(_bitPos++);
            const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

            items[i].level0    = 1;
            items[i].duration0 = half;
            items[i].level1    = 0;
            items[i].duration1 = half;
        }
    }

    void fillTask() {
        while (_running) {
            FillRequest req;
            if (xQueueReceive(_fillQueue, &req, portMAX_DELAY) == pdTRUE) {
                if (!_running || req.items == nullptr) break;
                fillSymbols(req.items, req.count);
            }
        }
    }

    static void fillTask_c(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->fillTask();
        vTaskDelete(nullptr);
    }

    // Called from ISR context — must not block, must be IRAM-resident.
    static void IRAM_ATTR fillCallback(volatile rmt_symbol_word_t *items,
                                       uint16_t count, void *ctx) {
        auto *self = static_cast<ESP32RMTChannel *>(ctx);
        if (!self->_running || !self->_fillQueue) return;

        const FillRequest req{items, count};
        BaseType_t woken = pdFALSE;
        // Overwrite any stale request — queue depth is 1.
        xQueueOverwriteFromISR(self->_fillQueue, &req, &woken);
        portYIELD_FROM_ISR(woken);
    }
};
}
