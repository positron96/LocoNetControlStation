#pragma once

#include "packet.hpp"
#include "LocoAddress.h"
#include "LocoSpeed.h"

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
     * Add different DCC packets to it, and it will pick packets when
     * DCC signal generator needs one.
     * Has priority selection mechanism, e.g.emergency stop packet will be prioritized over other packets.
     *
     * Decides if packets need to be refreshed periodically to tracks or not.
     * E.g. first 12 or so locomotive functions are sent periodically,
     *   while higher functions don't.
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
            it->second[0] = PacketBits::from_bytes(bytes);
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
            it->second[idx] = PacketBits::from_bytes(bytes);
            enqueue_slot_packet(SlotLocation{it, idx}, 0);
            return true;
        }

        bool put_generic_packet(etl::span<uint8_t> bytes, uint8_t nRepeats, int priority = 0) {
            if(queue_packets.full()) return false;
            QueueItem item{
                .priority = priority,
                .data = PacketWithRepeats{PacketBits::from_bytes(bytes), nRepeats}
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
                        slot_phase = (slot_phase+1) % MAX_PHASE;
                    }
                }
                loco_slots.erase(it);
            }
        }

        /**
         * Finds packet with highest priority and puts it into dst.
         * Updates its own storage, e.g. removed packet if it only needs to be sent once.
         */
        bool fetch_next_packet(PacketWithRepeats &dst) {
            if(!queue_packets.empty()) {
                QueueItem item = queue_packets.top();
                queue_packets.pop();
                if(etl::holds_alternative<PacketWithRepeats>(item.data)) {
                    dst = etl::get<PacketWithRepeats>(item.data);
                    return true;
                } else {
                    auto loc = etl::get<SlotLocation>(item.data);
                    if(loc.it->second[loc.idx].has_value() ) { // should always be true
                        dst = PacketWithRepeats{loc.it->second[loc.idx].value(), 1};
                        // update cur_slot and phase, it will be advanced on next call;
                        cur_slot = loc.it;
                        slot_phase = loc.idx == 0 ? 0 : loc.idx*2+1; // convert index to phase
                        return true;
                    }
                }
            }

            // get packet from loco slots in round-robin way
            cur_slot++;
            if(cur_slot == loco_slots.end()) {
                cur_slot = loco_slots.begin();
                slot_phase = (slot_phase+1) % MAX_PHASE;
            }
            // every even phase -> 0th (speed/dir) packet, every odd one -> fn packet
            size_t idx = slot_phase % 2 == 0 ? 0 : slot_phase / 2 + 1;
            if(cur_slot->second[idx].has_value() ) {
                dst = PacketWithRepeats{cur_slot->second[idx].value(), 1};
                return true;
            }

            return false;
        }
    protected:

        constexpr static size_t N_FNS_PER_LOCO = 3;
        constexpr static size_t N_PACKETS_PER_LOCO = N_FNS_PER_LOCO + 1;
        constexpr static size_t MAX_PHASE = N_PACKETS_PER_LOCO * 2;
        using LocoSlot = etl::array< etl::optional<PacketBits>, N_PACKETS_PER_LOCO>;
        using ISlotMap = etl::imap<LocoAddress, LocoSlot>;
        ISlotMap &loco_slots;

        struct SlotLocation {
            ISlotMap::iterator it;
            size_t idx;
        };
        constexpr static size_t N_QUEUE_PACKETS = 10;
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

        size_t slot_phase{0};
        ISlotMap::iterator cur_slot;

        BasePacketList(ISlotMap &loco_slots): loco_slots{loco_slots} {}

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
