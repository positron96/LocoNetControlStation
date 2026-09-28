#include "dccpp_proto_decoder.hpp"

#include "command_station.hpp"
#include "config.hpp"

#include <etl/string_utilities.h>
#include <etl/string_view.h>
#include <etl/to_arithmetic.h>
#include <etl/vector.h>

#define FILE_LOG_LEVEL LEVEL_WARN
#include "log.h"

#define FMT_SV(sv)  (int)((sv).length()), (sv).data()

namespace dccpp {

    template<typename T>
    bool parse(etl::string_view text, T &value) {
        auto parsed = etl::to_arithmetic<T>(text);
        if(!parsed) return false;
        value = parsed.value();
        return true;
    }

    LocoAddress fromInt(unsigned a) {
        if(a<=127) return LocoAddress::shortAddr(a);
        return LocoAddress::longAddr(a);
    }

    void split_tokens(etl::string_view in, etl::ivector<etl::string_view> &parts) {
        etl::optional<etl::string_view> token;
        while ((token = etl::get_token(in, " ", token, true))) {
            parts.emplace_back(token.value());
        }
    }

    void DccppStreamHandler::loop() {
        while(stream->available() > 0) {
            char c = stream->read();
            Serial.print(c);
            if(c=='\n' || c=='\r') {
                process_line(buf);
                buf.clear();
            } else {
                buf.push_back(c);
            }
        }
    }

    String sv_str(etl::string_view sv) {
        return String(sv.data(), sv.length());
    }

    void DccppStreamHandler::process_line(const etl::string_view line) {
        if(line.empty()) return;

        auto trimmed = etl::trim_view_whitespace(line);

        if(trimmed.size() < 3 || trimmed.front() != '<' || trimmed.back() != '>') {
            LOGE("DCC++ parse error: malformed brackets: '%.*s'", FMT_SV(trimmed));
            return;
        }

        auto inner = etl::trim_view_whitespace(trimmed.substr(1, trimmed.size() - 2));
        if(inner.empty()) {
            LOGE("DCC++ parse error: empty command body in '%.*s'", FMT_SV(trimmed));
            return;
        }

        etl::vector<etl::string_view, 8> parts{};
        split_tokens(inner, parts);
        const size_t count = parts.size();
        if(count == 0) {
            LOGE("DCC++ parse error: no tokens in '%.*s'", FMT_SV(inner));
            return;
        }

        const auto cmd = parts[0];

        #define CHECK(cond, msg, ...) do { \
            if(!(cond)) { \
                LOGE(msg "('%.*s')", ##__VA_ARGS__, FMT_SV(trimmed)); \
                return; \
            } } while(0)


        switch(cmd.front()) {
            case '0':
            case '1': {
                // power control: <0> or <1>
                bool v = cmd[0] == '1';
                String ret;
                if(count == 1) {
                    CS.getMainTrack()->setPower(v);
                    CS.getProgTrack()->setPower(v);
                    ret = String("<p") + cmd[0] + ">";
                    stream->println(ret);
                } else {
                    // DCC-EX command: <0|1 MAIN|PROG>
                    ret = String("<p") + cmd[0] + " " + sv_str(parts[1]) + ">";
                    if(parts[1] == "MAIN") {
                        CS.getMainTrack()->setPower(v);
                        stream->println(ret);
                    } else if(parts[1] == "PROG") {
                        CS.getProgTrack()->setPower(v);
                        stream->println(ret);
                    }
                }
                break;
            }
            case 'R': {
                // read byte on prog: <R CV CALLBACKNUM CALLBACKSUB>
                CHECK(count >= 4, "Invalid CV read command");
                unsigned cv = 0, callback_num = 0, callback_sub = 0;
                CHECK(parse<unsigned>(parts[1], cv) && parse<unsigned>(parts[2], callback_num) && parse<unsigned>(parts[3], callback_sub),
                    "Bad numeric args");

                auto ret = CS.readCVProg(cv);
                String t = String("<r ") + callback_num + " " + callback_sub + " " + (ret?(int)ret.value():-1) +">";
                stream->println(t);
                break;
            }
            case 'W': {
                // write byte on prog: <W CV VALUE CALLBACKNUM CALLBACKSUB>
                CHECK(count >= 5, "Invalid CV write command");

                unsigned cv = 0, value = 0, callback_num = 0, callback_sub = 0;
                CHECK(parse<unsigned>(parts[1], cv) && parse<unsigned>(parts[2], value) && parse<unsigned>(parts[3], callback_num) && parse<unsigned>(parts[4], callback_sub),
                    "Bad numeric args");
                auto ret = CS.writeCvProg(cv, value);
                String t = String("<r ") + callback_num + " " + callback_sub + " " + (ret?(int)value:-1) + ">";
                stream->println(t);
                break;
            }
            case 'w': {
                // write byte on main: <w CAB CV VALUE>
                CHECK(count >= 4, "Invalid CV write command");
                unsigned addr = 0, cv = 0, value = 0;
                CHECK(parse<unsigned>(parts[1], addr) && parse<unsigned>(parts[2], cv) && parse<unsigned>(parts[3], value),
                    "Bad numeric args");
                CS.writeCvMain(fromInt(addr), cv, value);
                break;
            }
            case 't': {
                // throttle: <t REGISTER CAB SPEED DIRECTION>
                CHECK(count >= 5, "Invalid throttle command");
                int speed = 0;
                unsigned reg = 0, cab = 0, direction = 0;
                CHECK(parse<unsigned>(parts[1], reg) && parse<unsigned>(parts[2], cab)
                    && parse<int>(parts[3], speed) && parse<unsigned>(parts[4], direction), "Bad numeric args");
                CHECK(CS.isSlotSupported(reg) && speed >= -1 && speed <= 126 && direction < 2, "Invalid throttle args");
                const auto loco = fromInt(cab);
                CHECK(loco.isValid(), "Invalid cab address");
                if(!CS.isSlotAllocated(reg)) CS.initLocoSlot(reg, loco);
                CS.setLocoSpeed(reg, speed < 0 ? SPEED_EMGR : LocoSpeed::fromDCC(speed, SpeedMode::S128));
                CS.setLocoDir(reg, direction);
                CS.setLocoSlotRefresh(reg, true);
                stream->println(String("<T ") + reg + " " + (speed < 0 ? 0 : speed) + " " + direction + ">");
                break;
            }
            case 'a': {
                // accessory command: <a ADDRESS SUBADDRESS ACTIVATE>
                CHECK(count >= 4, "Invalid accessory command");
                unsigned addr = 0, subaddr = 0, activate = 0;
                CHECK(parse<unsigned>(parts[1], addr) && parse<unsigned>(parts[2], subaddr)
                    && parse<unsigned>(parts[3], activate) && addr <= 511 && subaddr < 4 && activate < 2, "Bad accessory args");
                //TODO: broadcast to LocoNet as well
                CS.getMainTrack()->sendAccessory(dcc::AccessoryAddress::from9bit(addr, subaddr), activate != 0);
                break;
            }
            case 'f': {
                // cab function command: <f CAB BYTE1 [BYTE2]>

                unsigned cab = 0, byte1 = 0, byte2 = 0;
                CHECK(count >= 3 && parse<unsigned>(parts[1], cab) && parse<unsigned>(parts[2], byte1),
                    "Bad numeric args");
                if(count >= 4) CHECK(parse<unsigned>(parts[3], byte2), "Bad numeric args");

                const auto loco = fromInt(cab);
                CHECK(loco.isValid(), "Invalid cab address");

                const auto slot = CS.findOrAllocateLocoSlot(loco);
                CHECK(slot != 0, "No loco slot available");

                if(count == 3) {
                    switch(byte1 & 0xF0u) {
                        case 0x80u:
                        case 0x90u: {
                            const uint32_t f0_4 = ((byte1 & 0x10u) >> 4)
                                                | ((byte1 & 0x0Fu) << 1);
                            CS.setLocoFns(slot, dcc::fn_group::F0_4, f0_4);
                            break;
                        }
                        case 0xA0u: {
                            const uint32_t f9_12 = (static_cast<uint32_t>(byte1 & 0x0Fu) << 9);
                            CS.setLocoFns(slot, dcc::fn_group::F9_12, f9_12);
                            break;
                        }
                        case 0xB0u: {
                            const uint32_t f5_8 = (static_cast<uint32_t>(byte1 & 0x0Fu) << 5);
                            CS.setLocoFns(slot, dcc::fn_group::F5_8, f5_8);
                            break;
                        }
                        default:
                        CHECK(false, "Invalid function byte");
                    }
                } else {
                    switch(byte1) {
                        case 0xDEu: {
                            const uint32_t f13_20 = (static_cast<uint32_t>(byte2) << 13);
                            CS.setLocoFns(slot, dcc::fn_group::F13_20, f13_20);
                            break;
                        }
                        case 0xDFu: {
                            const uint32_t f21_28 = (static_cast<uint32_t>(byte2) << 21);
                            CS.setLocoFns(slot, dcc::fn_group::F21_28, f21_28);
                            break;
                        }
                        default:
                            CHECK(false, "Invalid extended function byte");
                    }
                }
                CS.setLocoSlotRefresh(slot, true);
                break;
            }
            case 'F': {
                // DCC-EX cab function command: <F CAB fn state>

                unsigned cab = 0, fn = 0, state = 0;
                CHECK(count>=4 && parse<unsigned>(parts[1], cab) && parse<unsigned>(parts[2], fn) && parse<unsigned>(parts[3], state),
                    "Bad numeric args");

                const auto loco = fromInt(cab);
                CHECK(loco.isValid(), "Invalid address");

                const auto slot = CS.findOrAllocateLocoSlot(loco);
                CHECK(slot != 0, "No loco slot available");
                CS.setLocoFn(slot, fn, state!=0);
                CS.setLocoSlotRefresh(slot, true);
                break;
            }
            case 's': {
                // Info: <s>
                char msg[30];
                for(const auto slot: CS.getAllocatedSlots()) {
                    const auto &data = CS.getSlotData(slot);
                    snprintf(msg, sizeof(msg), "<T%u %u %d>", slot, data.speed.get128(), data.dir ? 1 : 0);
                    stream->println(msg);
                }
                stream->println(String("<p") + (CS.getPowerState() ? "1>" : "0>"));
                stream->println(String("<i") + (CS_FULL_NAME " / " PCB_NAME ">"));
                break;
            }
            case 'c': {
                // DCC++ extension: <c CurrentMAIN {current} C Milli 0 {max_ma} 1 {trip_ma}>
                char msg[30];
                const auto &track = CS.getMainTrack();
                snprintf(msg, sizeof(msg), "<c CurrentMAIN %u C Milli 0 %u 1 %u>",
                    track->getCurrent(), track->getMaxCurrent(), track->getMaxCurrent()
                );
                stream->println(msg);
                break;
            }
            case '#': {
                // DCC++ extension: <#> Request number of supported cabs
                stream->println(String("<# ") + CommandStation::MAX_SLOTS + ">");
                break;
            }

            // Stubs

            case 'B': {
                // write bit on prog: <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
                // CV bit programming is unsupported; report verification failure.
                CHECK(count >= 6, "Invalid CV bit write command");
                stream->println(String("<r ") + sv_str(parts[4]) + " " + sv_str(parts[5]) + " -1>");
                break;
            }
            // write bit on main: < b CAB CV BIT VALUE >
            // CV bit programming is intentionally unsupported; documented response is none.
            case 'b': break;
            // Turnout commands <T ID ADDRESS SUBADDRESS>; <T ID>; <T>
            case 'T': { stream->println("<X>"); break; }
            case 'S': // create/edit/remove sensors
            case 'Q': { /* query sensors */ stream->println("<X>"); break; }
            // output pin commands: <Z ID STATE> | <Z ID PIN IFLAG>
            case 'Z': { stream->println("<X>"); break;}
            case 'E': {/* save eeprom */ stream->println("<e 0 0 0>"); break; }
            case 'e': { /* erase eeprom */ stream->println("<O>"); break; }
            case 'D': { /* diagnostics (ignored for now) */ break;}
            default:
                CHECK(false, "Unhandled command");
        }
        #undef CHECK
    }

}
