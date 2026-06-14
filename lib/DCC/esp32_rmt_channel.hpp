#pragma once

#include "esp32_channel.hpp"

#include <driver/rmt_common.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include <etl/array.h>
#include <etl/span.h>

// #include <driver/gpio.h>

namespace dcc {

    // constexpr int _debug_pin = 14;
    // constexpr int _debug_pin2 = 12;

/**
 * A DCC channel that outputs DCC waveform using ESP32 RMT TX peripheral.
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
        // pinMode(_debug_pin, OUTPUT);
        // pinMode(_debug_pin2, OUTPUT);
        ESP32Channel::begin();

        rmt_tx_channel_config_t txCfg{};
        txCfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        txCfg.clk_src = RMT_CLK_SRC_DEFAULT;
        txCfg.resolution_hz = 1000'000;  // 1 tick = 1us
        txCfg.mem_block_symbols = 64;
        txCfg.trans_queue_depth = 1; // !!! can be 2 (in zimo example)
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

        _running = true;
        if (xTaskCreate(packetTaskLoop_c, "dcc_rmt_tx", 4096, this, 1, &_packetTask) != pdPASS) {
            _running = false;
            DCC_LOGW("Failed to start DCC RMT task");
            return;
        }

    }

    void end() override {
        _running = false;

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

        ESP32Channel::end();
    }

private:
    static constexpr uint16_t DCC_ONE_HALF_US = 58;
    static constexpr uint16_t DCC_ZERO_HALF_US = 116;
    static constexpr size_t MAX_RMT_ITEMS = PacketBits::MAX_RAW_PACKET_BYTES * 8;

    etl::array<rmt_symbol_word_t, MAX_RMT_ITEMS> rmt_items;

    rmt_channel_handle_t _rmtChannel{nullptr};
    rmt_encoder_handle_t _copyEncoder{nullptr};

    volatile bool _running{false};
    TaskHandle_t _packetTask{nullptr};

    static void packetTaskLoop_c(void *arg) {
        static_cast<ESP32RMTChannel *>(arg)->packetTaskLoop();
        vTaskDelete(nullptr);
    }

    static size_t fillRmt(
        const Packet &packet,
        etl::span<rmt_symbol_word_t> items
    ) {

        if (items.size() == 0 || packet.size() == 0) {
            return 0;
        }

        PacketBits bits = PacketBits::from_packet(packet, DEF_PREAMBLE_LEN-1);

        // add one bit more so that last bit with currupted duration is not an end bit.
        // uint8_t *dd = bits.buf.data();
        // DCC_LOGI("bytes %d: %02X %02X %02X", packet.size(), packet[0],packet[1],packet[2]);
        // Serial.printf("bytes %s\r\n", fmt_span(packet));
        bits.append_bit(1);
        // DCC_LOGI("bits  %d: %02X %02X %02X %02X  %02X %02X %02X", bits.size_bits, dd[0],dd[1],dd[2], dd[3],dd[4],dd[5], dd[6]);

        // TODO: if packet does not fit, this just fills what it can. Maybe fail?
        size_t nBit = 0;
        for (nBit = 0; nBit < bits.size_bits && nBit < items.size(); ++nBit) {
            const bool isOne = bits.bit_at(nBit);
            const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

            rmt_symbol_word_t &item = items[nBit];
            item.level0 = 1;  item.duration0 = half;
            item.level1 = 0;  item.duration1 = half;
        }

        return nBit;
    }

    void packetTaskLoop() {
        PacketWithRepeats packet;

        while (_running) {
            // gpio_set_level(static_cast<gpio_num_t>(_debug_pin), 1);
            if (!packets.fetch_next_packet(packet)) {
                //DCC_LOGD_ISR("No packets pending, sending idle");
                packet = {idlePacket, 1};
            } else {
                DCC_LOGD("fetched: %sx%d", fmt_span(packet.packet), packet.nRepeats);
            }

            rmt_transmit_config_t tx_opts = {};
            tx_opts.flags.eot_level = 0; // set output low at end of transmission to match last pulse
            // gpio_set_level(static_cast<gpio_num_t>(_debug_pin), 0);

            const size_t itemCount = fillRmt(packet.packet, rmt_items);
            assert(itemCount>0);
            rmt_items[itemCount-1].duration1 -= 30; // compensate for latency before next transmission starts. TODO: to tune later.

            // gpio_set_level(static_cast<gpio_num_t>(_debug_pin), 1);
            for(size_t i=0; i<packet.nRepeats; i++) { // OG ESP32 doesn't support loop_count, so loop manually
                ESP_ERROR_CHECK(rmt_transmit(
                    _rmtChannel,
                    this->_copyEncoder,
                    rmt_items.data(),
                    itemCount * sizeof(rmt_symbol_word_t),
                    &tx_opts
                ));
            }
            // gpio_set_level(static_cast<gpio_num_t>(_debug_pin), 0);

        }
    }

};

}
