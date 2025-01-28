#ifndef NMEA0183_AIS_HPP
#define NMEA0183_AIS_HPP

#include <ais_base/Position.hpp>
#include <ais_base/VesselInformation.hpp>
#include <ais_base/VoyageInformation.hpp>
#include <marnav/ais/message.hpp>
#include <nmea0183/Driver.hpp>

#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_05.hpp>

#include <base/samples/RigidBodyState.hpp>
#include <gps_base/BaseTypes.hpp>
#include <gps_base/UTMConverter.hpp>
#include <optional>

namespace nmea0183 {
    class AIS {
        uint32_t mDiscardedSentenceCount = 0;
        Driver& mDriver;
        std::vector<std::pair<std::string, std::uint32_t>> payloads;

    public:
        AIS(Driver& driver);

        /** Read an AIS message
         *
         * This calls the underlying NMEA driver until a full
         * AIS message is received and returns it
         */
        std::unique_ptr<marnav::ais::message> readMessage();

        /**
         * Process a NMEA sentence and return an AIS message if one is
         * available
         *
         * AIS messages are made of multiple NMEA sentences. This adds a
         * sentence to the message reconstruction, and returns a full
         * AIS message once all fragments of one are received.
         *
         * The returned value will either be NULL or point to an AIS message
         */
        std::unique_ptr<marnav::ais::message> processSentence(
            marnav::nmea::sentence const& sentence);

        /** Returns the count of sentences that have been discarded because
         * of some reordering/reassembly issues
         */
        uint32_t getDiscardedSentenceCount() const;

        /**
         * Applies position correction using the vessel reference position and the sensor
         * offset
         *
         * @param position the AIS position to be corrected
         * @param sensor2vessel_in_vessel_pos The position of the sensor relative to the
         * vessel
         * @param utm_converter The UTM converter
         *
         * @return The corrected AIS position with updated latitude, longitude, and
         * correction status
         */
        static ais_base::Position applyPositionCorrection(
            ais_base::Position const& position,
            base::Vector3d const& sensor2vessel_in_vessel_pos,
            gps_base::UTMConverter utm_converter);

        static ais_base::Position getPosition(marnav::ais::message_01 const& message);
        static ais_base::VesselInformation getVesselInformation(
            marnav::ais::message_05 const& message);
        static ais_base::VoyageInformation getVoyageInformation(
            marnav::ais::message_05 const& message);
    };
}

#endif