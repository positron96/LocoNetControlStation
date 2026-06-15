#pragma once

#include "base_channel.hpp"
#include "esp32_channel.hpp"
#include <ulp_bitstream.hpp>

#include <etl/queue.h>
#include <etl/array.h>

/** Uses ESP32 ULP FSM to output bit streams. */

namespace dcc::esp32 {

    class ESP32UlpChannel : public dcc::ESP32Channel {
    public:
        ESP32UlpChannel(
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

        }

        void end() override {
            ESP32Channel::end();
        }

        friend class ESP32UlpManager;

    };

    class ESP32UlpManager {
    public:
        explicit ESP32UlpManager();

        void setMainChannel(ESP32UlpChannel * ch) { channels[0] = ch;}
        void setProgChannel(ESP32UlpChannel * ch) { channels[1] = ch;}

        void begin() {

            // for now assert both are set
            assert(channels[0] != nullptr && channels[1] != nullptr);

            ulp_bitstream::config_t cfg;
            cfg.gpio_a = static_cast<gpio_num_t>(channels[0]->_outputPin);
            cfg.gpio_b = static_cast<gpio_num_t>(channels[1]->_outputPin);
            cfg.buf_len = 100;
            cfg.timer_us = DCC_ONE_HALF_US;
            assert(ulp_bitstream::init(cfg) == 0);

            if (xTaskCreate(packetTaskLoop_c, "dcc_tx", 4096, this, 1, &_packetTask) != pdPASS) {
                _running = false;
                DCC_LOGW("Failed to start DCC RMT task");
                return;
            }

            _running = true;

        }

        void end() {
            _running = false;

            // if (_packetTask != nullptr) {
            //     vTaskDelete(_packetTask);
            //     _packetTask = nullptr;
            // }
        }

    private:

        static constexpr uint16_t DCC_ONE_HALF_US = 58;

        static constexpr size_t N = 2; // main and prog
        etl::array<ESP32UlpChannel*, N> channels{nullptr, nullptr};

        volatile bool _running{false};
        TaskHandle_t _packetTask{nullptr};

        static void packetTaskLoop_c(void *arg) {
            static_cast<ESP32UlpManager *>(arg)->packetTaskLoop();
            vTaskDelete(nullptr);
        }

        struct SymbolQueue {
            dcc::PacketBitsWithRepeats packet;
            size_t cur_bit;
            etl::queue<uint8_t, 4> symbols;
            bool next_packet_fetched{false};

            uint8_t next_symbol(dcc::BasePacketList *src) {
                next_packet_fetched = false;
                if(symbols.empty()) {
                    uint8_t bit = get_next(src);
                    if(bit) {
                        symbols.push(1); symbols.push(0);
                    } else {
                        symbols.push(1); symbols.push(1);
                        symbols.push(0); symbols.push(0);
                    }
                }
                uint8_t v = symbols.front(); symbols.pop();
                return v;
            }

            uint8_t get_next(dcc::BasePacketList *src) {
                if (cur_bit == packet.packet.size_bits) {
                    cur_bit = 0;
                    if (packet.nRepeats>0) {
                        packet.nRepeats--;
                        DCC_LOGD("repeat packet = %d", packet.nRepeats);
                    } else {
                        PacketWithRepeats bytes;
                        if(src == nullptr) {
                            return 1; // bit 1 is shorter, output it.
                        }
                        if(src->fetch_next_packet(bytes)) {
                            packet = PacketBitsWithRepeats::from_packet(bytes, DEF_PREAMBLE_LEN);
                        } else {
                            packet = {idle_packet_bits, 1};
                        }
                        next_packet_fetched = true;
                        DCC_LOGD("next packet: %d bits, %s",
                            packet.packet.size_bits, fmt_span(packet.packet.buf));
                    }
                }
                uint8_t v = packet.packet.bit_at(cur_bit);
                cur_bit++;
                return v;
            }
        };

        void packetTaskLoop() {
            SymbolQueue symbols[N];

            ulp_bitstream::start();
            while (_running) {
                // every 1ms check if we can put something into ulp buffer
                size_t packets_fetched[N]{0};

                size_t avail = ulp_bitstream::available();
                size_t count=0;
                while(avail>0 && _running) {
                    uint8_t bits[N];
                    for(size_t i=0; i<N; i++) {
                        bits[i] = symbols[i].next_symbol(
                            channels[i] != nullptr ? &channels[i]->packets : nullptr);
                        if(symbols[i].next_packet_fetched) {
                            packets_fetched[i]++;
                        }
                    }
                    ulp_bitstream::write(1, &bits[0], &bits[1]);
                    avail--;
                    count++;
                    if(packets_fetched[0]>1 && packets_fetched[1]>1) {
                        break; // at most 1 packet per loop.
                    }
                }
                DCC_LOGI("pushed %d periods", count);
                delay(1);

            }
            ulp_bitstream::stop();

        }

    };

}
