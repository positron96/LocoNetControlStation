#pragma once

#include "packet.hpp"
#include "LocoAddress.h"
#include "LocoSpeed.h"
#include "dcc_log.hpp"

#include <etl/map.h>
#include <etl/array.h>
#include <etl/optional.h>
#include <etl/priority_queue.h>
#include <etl/variant.h>

#include <cstdlib>
#include <cstdint>

namespace dcc {

    /**
     * Container for DCC packets.
     *
     * Users should add different DCC packets to it, and it will pick packets
     *   when DCC signal generator needs one.
     * Has priority selection mechanism, e.g. emergency stop packet will be prioritized over other packets.
     *
     * Decides if packets need to be refreshed periodically to tracks or not.
     * E.g. first 12 locomotive functions are sent periodically,
     *   while higher functions aren't.
     *
     * Internally, maintains a priority queue with packets that need to be sent once
     *   and a table of packets that need to be refreshed, indexed by Loco address.
     * Each table row cycles through its packets with speed-dir packet being emitted every even cycle,
     *   while odd cycles are given to function packets.
     *
     * Example of table:
     * ```
     * Loco Addr | Speed-dir  | F0..F4  | F5..F8 | F9..F12
     * ----------+------------+---------+--------+---------
     * 10        | 100F       | 0b1100  | -      | -
     * 11        | 0F         | -       | 0b1100 | -
     * 1234      | 10R        | 0b1111  | 0b1111 | 0b1111
     * ```
     *
     *
     **/
    class BasePacketList {
    public:

        size_t free_loco_slots() const { return loco_slots.available(); }

        bool has_loco(const LocoAddress addr) const {
            return loco_slots.find(addr) != loco_slots.end();
        }

        // TODO: for S14 mode, F0 bit is also needed
        bool put_loco_speed_dir_packet(const LocoAddress addr, const LocoSpeed speed, const SpeedMode mode, const bool fwd) {

            // find slot for loco
            auto it = loco_slots.find(addr);
            if(it == loco_slots.end() ) {
                if(loco_slots.full()) return false;
                it = loco_slots.insert(ETL_OR_STD::make_pair(addr, LocoSlot{})).first;
            }
            auto bytes = make_speed_dir_packet(addr, speed, mode, fwd);
            it->second.packets[0] = packet_from_bytes(bytes);
            enqueue_slot_packet(SlotLocation{it, 0}, speed.isEmgr() ? -100 : 0);
            return true;
        }

        bool put_loco_fn_packet(const LocoAddress addr, fn_group fg, uint32_t data) {
            auto bytes = make_fn_packet(addr, fg, data);

            size_t idx = fn_group_index(fg) + 1;
            if(idx > N_PACKETS_PER_LOCO) {
                // use non-slot packet for extra functions
                return put_generic_packet(bytes, FN_PACKET_REPEATS, 0);
            }
            // find slot for loco
            auto it = loco_slots.find(addr);
            if(it == loco_slots.end() ) {
                if(loco_slots.full()) return false;
                it = loco_slots.insert(ETL_OR_STD::make_pair(addr, LocoSlot{})).first;
            }
            it->second.packets[idx] = packet_from_bytes(bytes);
            enqueue_slot_packet(SlotLocation{it, idx}, 0);
            return true;
        }

        bool put_generic_packet(const etl::span<const uint8_t> bytes, uint8_t n_repeats, int priority = 0) {
            if(queue_packets.full()) return false;
            switch(bytes.size()) {
                case 1: DCC_LOGI("[len=1:%02X]x%d", bytes[0], n_repeats); break;
                case 2: DCC_LOGI("[len=2:%02X %02X]x%d", bytes[0], bytes[1], n_repeats); break;
                default: DCC_LOGI("[len=%d:%02X...]x%d", bytes.size(), bytes[0], n_repeats); break;
            }

            QueueItem item{
                .priority = priority,
                .data = PacketWithRepeats{packet_from_bytes(bytes), n_repeats}
            };
            queue_packets.push(item);
            return true;
        }

        bool put_accessory_packet(uint16_t addr11, bool thrown) {
            if(queue_packets.full()) return false;
            auto bytes = make_accessory_packet(addr11, thrown);
            return put_generic_packet(bytes, ACCESSORY_PACKET_REPEATS);
        }

        void clear_loco(const LocoAddress addr) {
            auto it = loco_slots.find(addr);
            if(it != loco_slots.end() ) {
                if(cur_slot == it) {
                    // if currently selected slot is being cleared, move to next one to avoid sending invalid packet
                    cur_slot++;
                    if(cur_slot == loco_slots.end()) {
                        cur_slot = loco_slots.begin();
                    }
                }
                loco_slots.erase(it);
            }
        }

        /**
         * Finds packet in the queue or table and puts it into dst.
         *
         * @return true if packet was put into dst, false if no packets available.
         */
        bool fetch_next_packet(PacketWithRepeats &packet_out) {

            // if priority queue is not empty, use it
            if(!queue_packets.empty()) {
                QueueItem item = queue_packets.top();
                queue_packets.pop();
                if(etl::holds_alternative<PacketWithRepeats>(item.data)) {
                    DCC_LOGD("Fetching queue item");
                    packet_out = etl::get<PacketWithRepeats>(item.data);
                    return true;
                } else {
                    auto loc = etl::get<SlotLocation>(item.data);
                    LocoSlot &slot = loc.it->second;
                    if(slot.packets[loc.idx].has_value() ) {
                        DCC_LOGD("Fetching queue location slot %d, idx %d",
                            std::distance(loco_slots.begin(), loc.it), loc.idx);
                        packet_out = PacketWithRepeats{slot.packets[loc.idx].value(), 1};
                        // update cur_slot and phase, it will be advanced on next call;
                        cur_slot = loc.it;
                        slot.phase = Phase::from_index(loc.idx);
                        return true;
                    } // must have been removed from slots, fall through
                }
            }

            // prevent null pointer dereference in case of empty slots
            if(loco_slots.empty()) return false;

            // get packet from loco slots in round-robin way
            cur_slot++;
            if(cur_slot == loco_slots.end()) {
                cur_slot = loco_slots.begin();
            }
            LocoSlot &slot = cur_slot->second;

            slot.phase.inc();
            size_t idx = slot.phase.to_index();
            if(!slot.packets[idx].has_value() ) {
                // index we want has no packet, advance index (with rollover)
                for(size_t i=0; i<N_PACKETS_PER_LOCO; i++) {
                    idx++; if(idx==N_PACKETS_PER_LOCO) idx=0;
                    if(slot.packets[idx].has_value()) { slot.phase = Phase::from_index(idx); break; } // found it
                }
            }
            assert(slot.packets[idx].has_value()); // slot allocated, but we didn't find packets it in

            DCC_LOGD("Fetching slot %d idx %d (ph %d)",
                std::distance(loco_slots.begin(), cur_slot),
                idx, slot.phase);
            packet_out = PacketWithRepeats{slot.packets[idx].value(), 1};
            return true;
        }
    protected:

        constexpr static size_t N_FN_GROUPS_PER_LOCO = 3;
        constexpr static size_t N_PACKETS_PER_LOCO = N_FN_GROUPS_PER_LOCO + 1;
        constexpr static size_t MAX_PHASE = N_PACKETS_PER_LOCO * 2;

        struct Phase {
            size_t phase{0};

            /// every even phase -> 0th (speed/dir) packet, every odd one -> fn packet
            size_t to_index() {
                return phase % 2 == 0 ? 0 : phase / 2 + 1;
            }

            /// convert index to phase
            static Phase from_index(size_t idx) {
                return {idx == 0 ? 0 : (idx*2 + 1)};
            }

            void inc() {
                phase ++;
                if(phase == BasePacketList::MAX_PHASE) phase = 0;
            }
        };

        /** A row in slots table */
        struct LocoSlot {
            etl::array< etl::optional<Packet>, N_PACKETS_PER_LOCO> packets;
            Phase phase;
        };

        using ISlotMap = etl::imap<LocoAddress, LocoSlot>;

        /** A 2D table with packets that need to be periodically refreshed. */
        ISlotMap &loco_slots;
        ISlotMap::iterator cur_slot;

        /** A location in slot table: a map iterator and an index in packet array. */
        struct SlotLocation {
            ISlotMap::iterator it; ///< slot location in the map
            size_t idx; ///< index in slot, 0=speed/dir, 1.. = fn packets
        };

        constexpr static size_t N_QUEUE_PACKETS = 10;

        /** Item for priority queue, either a packet or a location in the slot table. */
        struct QueueItem {
            int priority;
            etl::variant<
                PacketWithRepeats,
                SlotLocation
            > data;
            bool operator< (const QueueItem& other) const {
                return priority < other.priority;
            }
        };
        etl::priority_queue<QueueItem, N_QUEUE_PACKETS> queue_packets;

        BasePacketList(ISlotMap &loco_slots)
        : loco_slots{loco_slots}, cur_slot{loco_slots.end()}
        {
        }

        /** Put a slot location into priority queue. */
        bool enqueue_slot_packet(SlotLocation loc, int priority) {
            if(queue_packets.full()) return false;

            QueueItem item{
                .priority = priority,
                .data = loc
            };
            queue_packets.push(item);
            return true;
        }

    };

    /** Specific template with data, templated by size. */
    template<size_t NUM_SLOTS>
    class PacketList: public BasePacketList{
    public:
        PacketList(): BasePacketList(_loco_slots) {}

    private:
        using SlotMap = etl::map<LocoAddress, LocoSlot, NUM_SLOTS>;
        SlotMap _loco_slots;

    };

}
