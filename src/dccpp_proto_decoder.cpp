#include "dccpp_proto_decoder.hpp"

#define FILE_LOG_LEVEL LEVEL_WARN
#include "log.h"

#include <etl/string_utilities.h>
#include <etl/string_view.h>
#include <etl/to_arithmetic.h>
#include <etl/array.h>

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

    size_t split_tokens(etl::string_view in, etl::array<etl::string_view, 8> &parts) {
        size_t count = 0;
        etl::optional<etl::string_view> token;
        auto remaining = etl::trim_view_whitespace(in);

        while((token = etl::get_token(remaining, etl::whitespace<char>::value(), token, true)) && count < parts.size()) {
            parts[count++] = token.value();
        }

        return count;
    }

    void DccppStreamHandler::process_line(const etl::string_view line) {
        if(line.empty()) return;

        auto trimmed = etl::trim_view_whitespace(line);

        if(trimmed.size() < 3 || trimmed.front() != '<' || trimmed.back() != '>') {
            LOGE("DCC++ parse error: malformed bracketed command: '%.*s'", FMT_SV(trimmed));
            return;
        }

        auto inner = etl::trim_view_whitespace(trimmed.substr(1, trimmed.size() - 2));
        if(inner.empty()) {
            LOGE("DCC++ parse error: empty command body in '%.*s'", FMT_SV(trimmed));
            return;
        }

        etl::array<etl::string_view, 8> parts{};
        const size_t count = split_tokens(inner, parts);
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
            case 'r':
                break;
            default:
                LOGE("DCC++ parse error: unsupported command '%.*s'", FMT_SV(cmd));
                return;
        }

        switch(cmd.front()) {
            case '0':
            case '1': {
                // power control: <0> or <1>
                break;
            }
            case 'R':
            case 'r': {
                if(count < 4) {
                    LOGE("DCC++ parse error: invalid CV read command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned cv = 0, callback_num = 0, callback_sub = 0;
                if(!parse_uint(parts[1], cv) || !parse_uint(parts[2], callback_num) || !parse_uint(parts[3], callback_sub)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                (void)cv;
                (void)callback_num;
                (void)callback_sub;
                break;
            }
            case 'W':
            case 'w': {
                if(count < 5) {
                    LOGE("DCC++ parse error: invalid CV write command '%.*s'", FMT_SV(trimmed));
                    return;
                }
                unsigned cv = 0, value = 0, callback_num = 0, callback_sub = 0;
                if(!parse_uint(parts[1], cv) || !parse_uint(parts[2], value) || !parse_uint(parts[3], callback_num) || !parse_uint(parts[4], callback_sub)) {
                    LOGE("DCC++ parse error: bad numeric args in '%.*s'", FMT_SV(trimmed));
                    return;
                }
                (void)cv;
                (void)value;
                (void)callback_num;
                (void)callback_sub;
                break;
            }
            case 'B':
            case 'b': {
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
