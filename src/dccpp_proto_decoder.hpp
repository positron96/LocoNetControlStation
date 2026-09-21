#pragma once

#include <Arduino.h>
#include <Stream.h>


#include <etl/string.h>
#include <etl/string_view.h>
#include <etl/to_arithmetic.h>


/**
 * DCC++ protocol handler.
 *
 * @see https://github.com/DccPlusPlus/BaseStation/wiki/Commands-for-DCCpp-BaseStation
 **/

namespace dccpp {

    class DccppStreamHandler {
    public:
        explicit DccppStreamHandler(Stream *stream): stream{stream} {}

        void loop();

    private:
        Stream *stream;
        etl::string<100> buf;

        void process_line(const etl::string_view line);
    };

}
