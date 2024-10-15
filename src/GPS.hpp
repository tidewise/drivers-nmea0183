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
    class GPS {
    public:
        GPS(Driver& driver);

        gps_base::Position getPosition(marnav::nmea::rmc const& rmc,
            marnav::nmea::gsa const& gsa);

        gps_base::SolutionQuality getSolutionQuality(marnav::nmea::gsa const& gsa);

    private:
        static gps_base::GPS_SOLUTION_TYPES getPositionType(
            marnav::utils::optional<marnav::nmea::mode_indicator> mode);
        Driver& m_driver;
    };
}

#endif