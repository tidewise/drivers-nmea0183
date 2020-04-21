#ifndef NMEA0183_EXCEPTIONS_HPP
#define NMEA0183_EXCEPTIONS_HPP

#include <stdexcept>

namespace nmea0183 {
    struct MarnavParsingError : std::runtime_error {
        using std::runtime_error::runtime_error;
    };
}

#endif