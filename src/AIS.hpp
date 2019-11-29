#ifndef NMEA0183_AIS_HPP
#define NMEA0183_AIS_HPP

#include <marnav/ais/message.hpp>
#include <nmea0183/Driver.hpp>

namespace nmea0183 {
    class AIS {
        Driver& m_driver;

    public:
        AIS(Driver& driver);

        std::unique_ptr<marnav::ais::message> readMessage();
    };
}

#endif