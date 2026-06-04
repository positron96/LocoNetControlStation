#pragma once

#include "base_channel.hpp"

#include <driver/rmt_common.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

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
        BasePacketList &packets
    ) : BaseChannel{packets},
        _outputPin{outputPin},
        _enPin{enPin},
        _sensePin{sensePin}
    { }

    void begin() override {
        pinMode(_outputPin, OUTPUT);
        pinMode(_enPin, OUTPUT);
        digitalWrite(_outputPin, LOW);
        digitalWrite(_enPin, LOW);

        analogSetPinAttenuation(_sensePin, ADC_0db);

        rmt_tx_channel_config_t txCfg{};
        txCfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        txCfg.clk_src = RMT_CLK_SRC_DEFAULT;
        txCfg.resolution_hz = 1000'000;  // 1 tick = 1us
        txCfg.mem_block_symbols = 64;
        txCfg.trans_queue_depth = 4;
        txCfg.intr_priority = 0;

        if (rmt_new_tx_channel(&txCfg, &_rmtChannel) != ESP_OK) {
            DCC_LOGW("RMT new TX channel failed for GPIO %d", _outputPin);
            return;
        }

        rmt_copy_encoder_config_t encCfg{};
        if (rmt_new_copy_encoder(&encCfg, &_copyEncoder) != ESP_OK) {
            DCC_LOGW("RMT copy encoder creation failed");
            rmt_del_channel(_rmtChannel);
            _rmtChannel = nullptr;
            return;
        }

        // rmt_tx_event_callbacks_t cb {
        //     .on_trans_done = txDoneFunc
        // };
        // rmt_tx_register_event_callbacks(_rmtChannel, &cb, this);

        if (rmt_enable(_rmtChannel) != ESP_OK) {
            DCC_LOGW("RMT TX channel enable failed");
            rmt_del_encoder(_copyEncoder);
            _copyEncoder = nullptr;
            rmt_del_channel(_rmtChannel);
            _rmtChannel = nullptr;
            return;
        }

        if (xTaskCreate(packetTaskFunc, "dcc_rmt_tx", 4096, this, 3, &_packetTask) != pdPASS) {
            _running = false;
            DCC_LOGW("Failed to start DCC RMT task");
            return;
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

        if (_packetTask != nullptr) {
            vTaskDelete(_packetTask);
            _packetTask = nullptr;
        }
        if (_rmtChannel != nullptr) {
            rmt_disable(_rmtChannel);
        }
        if (_copyEncoder != nullptr) {
            rmt_del_encoder(_copyEncoder);
            _copyEncoder = nullptr;
        }
        if (_rmtChannel != nullptr) {
            rmt_del_channel(_rmtChannel);
            _rmtChannel = nullptr;
        }

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

    etl::array<rmt_symbol_word_t, MAX_RMT_ITEMS> rmt_items;

    rmt_channel_handle_t _rmtChannel{nullptr};
    rmt_encoder_handle_t _copyEncoder{nullptr};

    volatile bool _running{false};
    TaskHandle_t _packetTask{nullptr};
    esp_timer_handle_t _adcTimer{nullptr};

    static void packetTaskFunc(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->packetTaskLoop();
        vTaskDelete(nullptr);
    }

    static void adcTimerFunc(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->updateCurrent();
    }


    static size_t fillRmt(
        const PacketBits &packet,
        etl::span<rmt_symbol_word_t> items
    ) {

        if (packet.len == 0 || items.size() == 0) {
            return 0;
        }

        size_t itemIdx = 0;
        for (size_t bit = 0; bit < packet.len && itemIdx < items.size(); ++bit) {
            const bool isOne = packet.bit_at(bit);
            const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

            rmt_symbol_word_t &item = items[itemIdx++];
            item.level0 = 0;  item.duration0 = half;
            item.level1 = 1;  item.duration1 = half;
        }
        return itemIdx;
    }

    void packetTaskLoop() {
        PacketWithRepeats packet;

        while (_running) {
            if (!packets.fetch_next_packet(packet)) {
                packet = {idle_packet_bits, 1};
            }

            const size_t packetBits = packet.packet.len;
            if (packetBits == 0) {
                continue;
            }

            // -1 because nRepeats=1 means loop_count=0 (once)
            rmt_transmit_config_t tx_opts = {
                .loop_count = packet.nRepeats - 1
            };
            tx_opts.flags.eot_level = 1; // set output low at end of transmission to match last pulse

            const size_t itemCount = fillRmt(packet.packet, rmt_items);
            rmt_items[itemCount-1].duration1 -= 15; // compensate for latency before next transmission starts. TODO: to tune later.

            for(size_t i=0; i<packet.nRepeats; i++) {
                ESP_ERROR_CHECK(rmt_transmit(
                    _rmtChannel,
                    this->_copyEncoder,
                    rmt_items.data(),
                    itemCount * sizeof(rmt_symbol_word_t),
                    &tx_opts
                ));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(
                    _rmtChannel, portMAX_DELAY
                ));
            }

        }
    }

};

}
