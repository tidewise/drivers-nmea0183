#include "GPS.hpp"
#include <nmea0183/GPS.hpp>

using namespace nmea0183;
using namespace marnav;
using namespace std;
using namespace gps_base;

gps_base::GPS_SOLUTION_TYPES GPS::getPositionType(
    marnav::utils::optional<marnav::nmea::mode_indicator> mode)
{
    // TODO: check returns
    switch (*mode) {
        case marnav::nmea::mode_indicator::invalid:
            return GPS_SOLUTION_TYPES::INVALID;
        case marnav::nmea::mode_indicator::autonomous:
            return GPS_SOLUTION_TYPES::AUTONOMOUS_2D;
        case marnav::nmea::mode_indicator::differential:
            return GPS_SOLUTION_TYPES::DIFFERENTIAL;
        case marnav::nmea::mode_indicator::estimated:
            return GPS_SOLUTION_TYPES::INVALID;
        case marnav::nmea::mode_indicator::manual_input:
            return GPS_SOLUTION_TYPES::INVALID;
        case marnav::nmea::mode_indicator::simulated:
            return GPS_SOLUTION_TYPES::INVALID;
        case marnav::nmea::mode_indicator::data_not_valid:
            return GPS_SOLUTION_TYPES::NO_SOLUTION;
        case marnav::nmea::mode_indicator::precise:
            return GPS_SOLUTION_TYPES::INVALID;
        default:
            return GPS_SOLUTION_TYPES::INVALID;
    }
}

gps_base::Position nmea0183::GPS::getPosition(marnav::nmea::rmc const& rmc,
    marnav::nmea::gsa const& gsa)
{
    gps_base::Position position;
    // TODO: time now or message time?
    position.time = base::Time::now();
    position.latitude = rmc.get_latitude().value();
    position.longitude = rmc.get_longitude().value();
    auto mode_indicator = rmc.get_mode_ind().value();
    position.positionType = getPositionType(mode_indicator);
    position.noOfSatellites = 0;
    for (int i = 0; i < gsa.max_satellite_ids; i++) {
        if (gsa.get_satellite_id(i).has_value()) {
            position.noOfSatellites += 1;
        }
    }
    return position;
}

gps_base::SolutionQuality nmea0183::GPS::getSolutionQuality(marnav::nmea::gsa const& gsa)
{
    gps_base::SolutionQuality solution_quality;
    solution_quality.pdop = gsa.get_pdop().value();
    solution_quality.hdop = gsa.get_hdop().value();
    solution_quality.vdop = gsa.get_vdop().value();
    for (int i = 0; i < gsa.max_satellite_ids; i++) {
        auto satellite_id = gsa.get_satellite_id(i);
        if (satellite_id.has_value()) {
            // TODO: check - used_satellite = satellite id?
            solution_quality.usedSatellites.push_back(satellite_id.value());
        }
    }
    return solution_quality;
}
