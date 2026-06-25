#pragma once

#include "base_channel.hpp"
#include "esp32_channel.hpp"

#include <rmt_cont.h>

#include <etl/array.h>
#include <etl/span.h>


namespace dcc {

/**
 * A DCC channel that outputs DCC waveform using ESP32 RMT TX peripheral.
 *
 * Core idea is taken from DCC-EX (DCCRMT.cpp):
 *   . split packets into preamble and payload,
 *   . put preamble at the beginning of the TX block and don't touch it,
 *   . start outputting in loop mode,
 *   . in TX DONE interrupt rewrite only payload area.
 *     RMT is outputting preamble while core is filling payload,
 *     so there are no gaps.
 *
 * DCC packet always starts at the beginning of the RMT block,
 *   as a consequence it must fully fit into RMT memory.
 * Not a problem for OG ESP32 (it has 8 channels 64 bits each, which can be combined),
 *   somewhat a problem for ESP32-S2/S3 (4 TX blocks 48 bits each),
 *   but definitely a problem for ESP32-C3/C5/C6 where there are only 2 TX blocks 48 bits each.
 *
 * On memory blocks:
 *   https://docs.espressif.com/projects/arduino-esp32/en/latest/api/rmt.html#rmt-memory-blocks
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

        //DCC_LOGI("RMT channel %d", _rmtChannel);
        rmt_config_t cfg{};
        cfg.rmt_mode = RMT_MODE_TX;
        cfg.channel = _rmtChannel;
        cfg.clk_div = APB_CLK_FREQ  / 1'000'000;
        cfg.gpio_num = static_cast<gpio_num_t>(_outputPin);
        cfg.mem_block_num = MEM_BLOCKS;
        ESP_ERROR_CHECK(rmt_config(&cfg));

        // NOTE: no ESP_INTR_FLAG_IRAM here
        ESP_ERROR_CHECK(rmt_driver_install(
            cfg.channel, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));
        ESP_ERROR_CHECK(rmt_set_tx_loop_mode(_rmtChannel, true));

        _channelObjects[_rmtChannel] = this;
        rmt_register_tx_end_callback(rmtTxDone_c, nullptr);
        rmt_set_tx_intr_en(_rmtChannel, true);

        const size_t itemCount = fillRmt(PacketBits::from_packet(idlePacket, PREAMBLE_BITS), rmt_items);

        // put whole idle packet, its preamble will be kept forever
        rmt_fill_tx_items(_rmtChannel, rmt_items.data(), itemCount, 0);

        rmt_tx_start(_rmtChannel, true);

        // _running = true;
        // if (xTaskCreatePinnedToCore(txTaskFunc, "dcc_rmt_tx", 4096, this, 1, &_txTask, 1) != pdPASS) {
        //     _running = false;
        //     DCC_LOGW("Failed to start DCC RMT task");
        // }

    }

    void end() override {
        // _running = false;
        // if (_txTask != nullptr) {
        //     //vTaskDelete(_txTask);
        //     _txTask = nullptr;
        // }

        _channelObjects[_rmtChannel] = nullptr;
        rmt_tx_stop(_rmtChannel);
        rmt_driver_uninstall(_rmtChannel);


        ESP32Channel::end();
    }

private:
    static constexpr uint16_t DCC_ONE_HALF_US = 58;
    static constexpr uint16_t DCC_ZERO_HALF_US = 100;
    static constexpr size_t MEM_BLOCKS = 2;
    static constexpr size_t MAX_RMT_ITEMS = SOC_RMT_MEM_WORDS_PER_CHANNEL * MEM_BLOCKS; // can't go above this
    static constexpr size_t PREAMBLE_BITS = DEF_PREAMBLE_LEN - 1; // for some reason, one extra bit it output first

    etl::array<rmt_item32_t, MAX_RMT_ITEMS> rmt_items;

    /** Used in ISR that is shared between channels. */
    inline static etl::array<ESP32RMTChannel *, SOC_RMT_CHANNELS_PER_GROUP> _channelObjects{nullptr};
    inline static uint8_t _channelCount{0};
    static uint8_t incChannel() {
        uint8_t t = _channelCount;
        _channelCount+=MEM_BLOCKS;
        return t;
    }
    rmt_channel_t _rmtChannel{static_cast<rmt_channel_t>(incChannel())}; // auto-increment for now.

    // volatile bool _running{false};
    // TaskHandle_t _txTask{nullptr};

    // static void txTaskFunc(void *arg) {
    //     static_cast<ESP32RMTChannel *>(arg)->txTaskLoop();
    //     vTaskDelete(nullptr);
    // }

    static size_t fillRmt(
        const PacketBits &packet,
        etl::span<rmt_item32_t> items
    ) {

        if (packet.size_bits == 0 || items.size() == 0) {
            return 0;
        }

        size_t maxItems = items.size() - 1; // account for stop symbol in the end
        size_t itemIdx = 0;
        for (size_t bit = 0; bit < packet.size_bits; ++bit) {
            if(itemIdx == maxItems) return 0; // for now, skip packet if it's too big.
            const bool isOne = packet.bit_at(bit);
            const uint16_t half = isOne ? DCC_ONE_HALF_US : DCC_ZERO_HALF_US;

            rmt_item32_t item{};
            item.level0 = 0;
            item.duration0 = half;
            item.level1 = 1;
            item.duration1 = half;
            items[itemIdx++] = item;
        }

        items[itemIdx++].val = 0; // stop symbol
        return itemIdx;
    }

    // void txTaskLoop() {
    //     PacketWithRepeats packet;

    //     while (_running) {
    //         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //         // Serial.println("Task");

    //         if (repeatsLeft>0) {
    //             repeatsLeft--;
    //             // DCC_LOGI("repeat packet = %d", repeatsLeft);
    //         } else {
    //             PacketWithRepeats packet;
    //             if (!packets.fetch_next_packet(packet)) {
    //                 packet = {idlePacket, 1};
    //             }

    //             // note 0 preamble bits here!
    //             // also limit number of bits that can be written
    //             const size_t itemCount = fillRmt(
    //                 PacketBits::from_packet(packet.packet, 0),
    //                 {rmt_items.begin(), rmt_items.size() - PREAMBLE_BITS});
    //             assert(itemCount!=0);

    //             // DCC_LOGI("next packet: %s*%d",
    //             //     fmt_span(packet.packet),
    //             //     packet.nRepeats);
    //             // this is data without preamble, with offset.
    //             rmt_fill_tx_items(_rmtChannel, rmt_items.data(), itemCount, PREAMBLE_BITS);
    //         }


    //     }

    //     gpio_set_level(static_cast<gpio_num_t>(_outputPin), 0);
    // }

    static IRAM_ATTR void rmtTxDone_c(rmt_channel_t channel, void *arg) {
        if(_channelObjects[channel]) {
            _channelObjects[channel]->rmtTxDoneCallback();
        }
    }

    uint8_t repeatsLeft{0};

    IRAM_ATTR void rmtTxDoneCallback() {
        // ets_printf("RMT channel %d, len %d\n", _rmtChannel, rmt_items.size());
        // if(_running && _txTask != nullptr) {
        //     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        //     vTaskNotifyGiveFromISR(_txTask, &xHigherPriorityTaskWoken);
        //     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        // }
        if (repeatsLeft>0) {
            repeatsLeft--;
            DCC_LOGD_ISR("repeat packet = %d", repeatsLeft);
            // don't do anything, RMT will repeat on its own
        } else {
            PacketWithRepeats packet;
            if (!packets.fetch_next_packet(packet)) {
                packet = {idlePacket, 1};
            }

            // note 0 preamble bits here!
            // also limit number of bits that can be written
            const size_t itemCount = fillRmt(
                PacketBits::from_packet(packet.packet, 0),
                {rmt_items.begin(), rmt_items.size() - PREAMBLE_BITS});
            assert(itemCount!=0);

            DCC_LOGD_ISR("next packet: [%d]=[%02X %02X...]*%d",
                packet.packet.size_bits,
                packet.packet.buf[0], packet.packet.buf[1],
                packet.nRepeats);
            // this is data without preamble, put with offset.
            rmt_fill_tx_items(_rmtChannel, rmt_items.data(), itemCount, PREAMBLE_BITS);
        }

    }
};

}
