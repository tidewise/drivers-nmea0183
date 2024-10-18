#ifndef NMEA0183_GPS_HPP
#define NMEA0183_GPS_HPP

#include <gps_base/BaseTypes.hpp>
#include <marnav/nmea/gsa.hpp>
#include <marnav/nmea/rmc.hpp>
#include <nmea0183/Driver.hpp>

namespace nmea0183 {
    /**
     * @brief Parses nmea 0183 messages into gps objects
     *
     */
    namespace GPS {
        /**
         * @brief Get the Solution object from the nmea 0183 rmc and gsa messages
         *
         * @param rmc The rmc message
         * @param gsa The gsa message
         * @return gps_base::Solution
         */
        gps_base::Solution getSolution(marnav::nmea::rmc const& rmc,
            marnav::nmea::gsa const& gsa);
        /**
         * @brief Get the Solution Quality object from the nmea 0183 gsa message
         *
         * @param gsa The gsa message
         * @return gps_base::SolutionQuality
         */
        gps_base::SolutionQuality getSolutionQuality(marnav::nmea::gsa const& gsa);
        /**
         * @brief Get the Position Type object from the nmea 0183 mode indicator
         *
         * @param mode The mode indicator
         * @return gps_base::GPS_SOLUTION_TYPES
         */
        gps_base::GPS_SOLUTION_TYPES getPositionType(marnav::nmea::mode_indicator mode);
    };
}

#endif