#ifndef NMEA0183_GPS_HPP
#define NMEA0183_GPS_HPP

#include <gps_base/BaseTypes.hpp>
#include <marnav/nmea/gsa.hpp>
#include <marnav/nmea/rmc.hpp>
#include <nmea0183/Driver.hpp>

namespace nmea0183 {
    /**
     *
     */
    namespace GPS {
        gps_base::Position getPosition(marnav::nmea::rmc const& rmc,
            marnav::nmea::gsa const& gsa);

        gps_base::SolutionQuality getSolutionQuality(marnav::nmea::gsa const& gsa);

        gps_base::GPS_SOLUTION_TYPES getPositionType(
            marnav::utils::optional<marnav::nmea::mode_indicator> mode);
        base::Time buildRockTime(
            marnav::utils::optional<marnav::nmea::time> const& optional_time,
            marnav::utils::optional<marnav::nmea::date> const& optional_date);
    };
}

#endif