#pragma once

#include "base_channel.hpp"

#include <driver/rmt.h>

#include <etl/array.h>
#include <etl/span.h>


namespace dcc {

/**
 * A DCC channel that outputs DCC waveform using ESP32 RMT TX peripheral.
 */
class ESP32RMTChannel : public BaseChannel {
public:
    ESP32RMTChannel(
        uint8_t outputPin,
        uint8_t enPin,
        uint8_t sensePin,
        BasePacketList &packets,
        rmt_channel_t rmtChannel = RMT_CHANNEL_0
    ) : BaseChannel{packets},
        _outputPin{outputPin},
        _enPin{enPin},
        _sensePin{sensePin},
        _rmtChannel{rmtChannel}
    { }

    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_outputPin, LOW);
        digitalWrite(_enPin, LOW);

        analogSetPinAttenuation(_sensePin, ADC_0db);

        rmt_config_t cfg{};
        cfg.rmt_mode = RMT_MODE_TX;
        cfg.channel = _rmtChannel;
        cfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        cfg.mem_block_num = 1;
        cfg.clk_div = 80;  // 1 tick = 1us (80MHz / 80)
        cfg.tx_config.loop_en = false;
        cfg.tx_config.carrier_en = false;
        cfg.tx_config.idle_output_en = true;
        cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

        if (rmt_config(&cfg) != ESP_OK) {
            DCC_LOGW("RMT config failed for GPIO %d", _outputPin);
            return;
        }
        if (rmt_driver_install(_rmtChannel, 0, 0) != ESP_OK) {
            DCC_LOGW("RMT driver install failed for channel %d", static_cast<int>(_rmtChannel));
            return;
        }

        _running = true;
        if (xTaskCreate(txTaskFunc, "dcc_rmt_tx", 4096, this, 2, &_txTask) != pdPASS) {
            _running = false;
            DCC_LOGW("Failed to start DCC RMT task");
        }

        esp_timer_create_args_t adcCfg{adcTimerFunc, this, ESP_TIMER_TASK, "dcc_adc"};
        if (esp_timer_create(&adcCfg, &_adcTimer) == ESP_OK) {
            esp_timer_start_periodic(_adcTimer, 1000);  // 1ms
        }
    }

    void end() override {
        _running = false;

        if (_adcTimer != nullptr) {
            esp_timer_stop(_adcTimer);
            esp_timer_delete(_adcTimer);
            _adcTimer = nullptr;
        }

        if (_txTask != nullptr) {
            rmt_tx_stop(_rmtChannel);
            vTaskDelete(_txTask);
            _txTask = nullptr;
        }

        rmt_driver_uninstall(_rmtChannel);

        pinMode(_outputPin, INPUT);
        pinMode(_enPin, INPUT);
    }

    void setPower(bool v) override {
        DCC_LOGI("setPower(%d)", v);
        digitalWrite(_enPin, v ? HIGH : LOW);
    }

    bool getPower() override {
        return digitalRead(_enPin) == HIGH;
    }

    void updateCurrent() override {
        uint16_t c = analogRead(_sensePin);
        current = c;
        if (c > maxCurrent) {
            maxCurrent = c;
        }
    }

private:
    static constexpr uint16_t DCC_ONE_HALF_US = 58;
    static constexpr uint16_t DCC_ZERO_HALF_US = 116;
    static constexpr size_t MAX_RMT_ITEMS = PacketBits::MAX_RAW_PACKET_BYTES * 8;

    uint8_t _outputPin;
    uint8_t _enPin;
    uint8_t _sensePin;
    rmt_channel_t _rmtChannel;
    etl::array<rmt_item32_t, MAX_RMT_ITEMS> rmt_items;

    volatile bool _running{false};
    TaskHandle_t _txTask{nullptr};
    esp_timer_handle_t _adcTimer{nullptr};

    static void txTaskFunc(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->txTaskLoop();
        vTaskDelete(nullptr);
    }

    static void adcTimerFunc(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->updateCurrent();
    }

    static size_t fillRmt(
        const PacketBits &packet,
        size_t repeats,
        etl::span<rmt_item32_t> items
    ) {

        if (packet.len == 0 || items.size() == 0) {
            return 0;
        }

        size_t itemIdx = 0;
        for (size_t rep = 0; rep < repeats && itemIdx < items.size(); ++rep) {
            for (size_t bit = 0; bit < packet.len && itemIdx < items.size(); ++bit) {
                const bool isOne = packet.bit_at(bit);
                const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

                rmt_item32_t item{};
                item.level0 = 0;
                item.duration0 = half;
                item.level1 = 1;
                item.duration1 = half;
                items[itemIdx++] = item;
            }
        }
        return itemIdx;
    }

    void txTaskLoop() {
        PacketWithRepeats packet;

        while (_running) {
            if (!packets.fetch_next_packet(packet)) {
                packet = {idle_packet_bits, 1};
            }

            const size_t packetBits = packet.packet.len;
            if (packetBits == 0) {
                continue;
            }

            uint8_t repeatsLeft = packet.nRepeats;
            //const size_t chunkRepeats = 1; // ETL_OR_STD::min(repeatsLeft, maxRepeatsInChunk);
            const size_t itemCount = fillRmt(packet.packet, 1, rmt_items);
            // const size_t maxRepeatsInChunk = ETL_OR_STD::max<size_t>(1, MAX_RMT_ITEMS / packetBits);

            while (_running && repeatsLeft > 0) {
                if (itemCount == 0) {
                    break;
                }

                // blocking implementation for now.
                const esp_err_t err = rmt_write_items(_rmtChannel, rmt_items.data(), itemCount, true);
                if (err != ESP_OK) {
                    DCC_LOGW("rmt_write_items failed (%d)", static_cast<int>(err));
                    _running = false;
                    break;
                }

                repeatsLeft -= 1;
            }
        }

        gpio_set_level(static_cast<gpio_num_t>(_outputPin), 0);
    }
};

}
