#include "dccpp_proto_decoder.hpp"

#include "command_station.hpp"

#include <etl/string_utilities.h>
#include <etl/string_view.h>
#include <etl/to_arithmetic.h>
#include <etl/vector.h>

#define FILE_LOG_LEVEL LEVEL_WARN
#include "log.h"

#define FMT_SV(sv)  (int)((sv).length()), (sv).data()

namespace dccpp {

    bool parse_int(etl::string_view text, int &value) {
        auto parsed = etl::to_arithmetic<int>(text);
        if(!parsed) return false;
        value = parsed.value();
        return true;
    }

    bool parse_uint(etl::string_view text, unsigned &value) {
        auto parsed = etl::to_arithmetic<unsigned>(text);
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

        switch(cmd.front()) {
            case '0':
            case '1':
            case 's':
            case 'S':
            case 'Q':
            case 'q':
            case 'T':
            case 't':
            case 'E':
            case 'e':
            case 'D':
            case 'd':
            case 'Z':
            case 'z':
            case 'a':
            case 'f':
            case 'w':
            case 'W':
            case 'b':
            case 'B':
            case 'R':
                break;
            default:
                LOGE("DCC++ parse error: unsupported command '%.*s'", FMT_SV(cmd));
                return;
        }

        switch(cmd.front()) {
            case '0':
            case '1': {
                // power control: <0> or <1>
                bool v = cmd[0] == '1';
                if(parts.size()==1) {
                    CS.getMainTrack()->setPower(v);
                    CS.getProgTrack()->setPower(v);
                } else {
                    if(parts[1] == "MAIN") {
                        CS.getMainTrack()->setPower(v);
                    } else if(parts[1] == "PROG") {
                        CS.getProgTrack()->setPower(v);
                    }
                }
                break;
            }
            case 'R': {
                // read byte on prog: < R CV CALLBACKNUM CALLBACKSUB >
                if(count < 4) {
                    LOGE("DCC++ parse error: invalid CV read command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned cv = 0, callback_num = 0, callback_sub = 0;
                if(!parse_uint(parts[1], cv) || !parse_uint(parts[2], callback_num) || !parse_uint(parts[3], callback_sub)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                auto ret = CS.readCVProg(cv);
                String t = String("<r ") + callback_num + " " + callback_sub + " " + (ret?(int)ret.value():-1) +">";
                stream->println(t);
                break;
            }
            case 'W': {
                // write byte on prog: < W CV VALUE CALLBACKNUM CALLBACKSUB >
                if(count < 5) {
                    LOGE("DCC++ parse error: invalid CV write command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned cv = 0, value = 0, callback_num = 0, callback_sub = 0;
                if(!parse_uint(parts[1], cv) || !parse_uint(parts[2], value) || !parse_uint(parts[3], callback_num) || !parse_uint(parts[4], callback_sub)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                auto ret = CS.writeCvProg(cv, value);
                String t = String("<r ") + callback_num + " " + callback_sub + " " + (ret?(int)value:-1) + ">";
                stream->println(t);
                break;
            }
            case 'w': {
                // write byte on main: < w CAB CV VALUE >
                if(count < 4) {
                    LOGE("DCC++ parse error: invalid CV write command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned addr = 0, cv = 0, value = 0;
                if(!parse_uint(parts[1], addr) || !parse_uint(parts[2], cv) || !parse_uint(parts[3], value)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                CS.writeCvMain(fromInt(addr), cv, value);
                stream->println("OK");
                break;
            }
            case 'B': {
                // write bit on prog: < B CV BIT VALUE CALLBACKNUM CALLBACKSUB >
                if(count < 6) {
                    LOGE("DCC++ parse error: invalid CV bit write command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned cv = 0, bit = 0, value = 0, callback_num = 0, callback_sub = 0;
                if(!parse_uint(parts[1], cv) || !parse_uint(parts[2], bit) || !parse_uint(parts[3], value) || !parse_uint(parts[4], callback_num) || !parse_uint(parts[5], callback_sub)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                (void)cv;
                (void)bit;
                (void)value;
                (void)callback_num;
                (void)callback_sub;
                break;
            }
            case 'b':
                // write bit on main: < b CAB CV BIT VALUE >
                break;
            case 'T':
            case 't': {
                // turnout listing/define/control: <T> | <T ID> | <T ID THROW> | <T ID ADDRESS SUBADDRESS>
                break;
            }
            case 'a': {
                // accessory command: <a ADDRESS SUBADDRESS ACTIVATE>
                break;
            }
            case 'f': {
                // cab function command: <f CAB BYTE1 [BYTE2]>
                if(count < 3) {
                    LOGE("DCC++ parse error: invalid cab function command '%.*s'", FMT_SV(trimmed));
                    return;
                }

                unsigned cab = 0, byte1 = 0, byte2 = 0;
                if(!parse_uint(parts[1], cab) || !parse_uint(parts[2], byte1)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                if(count >= 4 && !parse_uint(parts[3], byte2)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }

                const auto loco = fromInt(cab);
                if(!loco.isValid()) {
                    LOGE("DCC++ parse error: invalid cab address in '%.*s'", FMT_SV(trimmed));
                    return;
                }

                const auto slot = CS.findOrAllocateLocoSlot(loco);
                if(slot == 0) {
                    LOGE("DCC++ parse error: no loco slot available for '%.*s'", FMT_SV(trimmed));
                    return;
                }
                CS.setLocoSlotRefresh(slot, true);

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
                            LOGE("DCC++ parse error: invalid function byte in '%.*s'", FMT_SV(trimmed));
                            return;
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
                            LOGE("DCC++ parse error: invalid extended function byte in '%.*s'", FMT_SV(trimmed));
                            return;
                    }
                }
                break;
            }
            case 's':
            case 'S':
            case 'Q':
            case 'q': {
                // status/list/query commands
                break;
            }
            case 'Z':
            case 'z': {
                // output pin commands: <Z ID STATE> | <Z ID PIN IFLAG>
                break;
            }
            case 'E':
            case 'e': {
                // save/erase EEPROM
                break;
            }
            case 'D':
            case 'd': {
                // diagnostics (ignored here for now)
                break;
            }
            default:
                LOGE("DCC++ parse error: unhandled command '%.*s'", FMT_SV(cmd));
                return;
        }
    }

}
