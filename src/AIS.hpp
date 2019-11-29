#ifndef NMEA0183_AIS_HPP
#define NMEA0183_AIS_HPP

#include <marnav/ais/message.hpp>
#include <nmea0183/Driver.hpp>
#include <ais_base/Position.hpp>
#include <ais_base/VesselInformation.hpp>
#include <ais_base/VoyageInformation.hpp>

#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_05.hpp>

namespace nmea0183 {
    class AIS {
        Driver& m_driver;

    public:
        AIS(Driver& driver);

        std::unique_ptr<marnav::ais::message> readMessage();

        static ais_base::Position getPosition(
            marnav::ais::message_01 const& message);
        static ais_base::VesselInformation getVesselInformation(
            marnav::ais::message_05 const& message);
        static ais_base::VoyageInformation getVoyageInformation(
            marnav::ais::message_05 const& message);
    };
}

#endif