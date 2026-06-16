#pragma once

#include "esp32_channel.hpp"

#include <esp_attr.h>
#include <rmt_cont_tx.h>

namespace dcc {

/**
 * A DCC channel that outputs DCC waveform using continuous ESP32 RMT LL API.
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

        rmt_cont_tx_config_t cfg{};
        cfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        cfg.channel = _channel;
        cfg.mem_blocks = 1;
        cfg.resolution_hz = 1'000'000; // 1 tick = 1us
        cfg.idle_level = 0;
        cfg.fill_cb = &fillCallback;
        cfg.ctx = this;

        if (rmt_cont_tx_start(&cfg) != ESP_OK) {
            DCC_LOGW("RMT continuous TX start failed on GPIO %d (ch %d)", _outputPin, _channel);
            return;
        }

        _running = true;
    }

    void end() override {
        _running = false;
        if (_channel < 8) {
            rmt_cont_tx_stop(_channel);
        }
        ESP32Channel::end();
    }

private:
    static constexpr uint16_t DCC_ONE_HALF_US = 58;
    static constexpr uint16_t DCC_ZERO_HALF_US = 116;

    inline static uint8_t s_nextChannel{0};

    uint8_t _channel{s_nextChannel++};
    volatile bool _running{false};

    PacketBitsWithRepeats _activePacket{};
    size_t _bitPos{0};

    void IRAM_ATTR fillSymbols(volatile rmt_symbol_word_t *items, uint16_t count) {
        if (!_running || count == 0) {
            return;
        }

        for (uint16_t i = 0; i < count; ++i) {

            if(_bitPos >= _activePacket.packet.size_bits) {
                _bitPos = 0;
                if(_activePacket.nRepeats > 0) {
                    --_activePacket.nRepeats;
                } else {
                    PacketWithRepeats bytes;
                    if(packets.fetch_next_packet(bytes)) {
                        _activePacket = PacketBitsWithRepeats::from_packet(bytes, DEF_PREAMBLE_LEN);
                    } else {
                        _activePacket = {idle_packet_bits, 1};
                    }
                }
            }


            const bool isOne = _activePacket.packet.bit_at(_bitPos++);
            const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

            items[i].level0 = 1;
            items[i].duration0 = half;
            items[i].level1 = 0;
            items[i].duration1 = half;
        }
    }

    static void IRAM_ATTR fillCallback(volatile rmt_symbol_word_t *items, uint16_t count, void *ctx) {
        static_cast<ESP32RMTChannel *>(ctx)->fillSymbols(items, count);
    }
};

}
