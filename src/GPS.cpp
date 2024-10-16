#include "GPS.hpp"
#include <nmea0183/GPS.hpp>

using namespace nmea0183;
using namespace marnav;
using namespace std;
using namespace gps_base;

GPS_SOLUTION_TYPES GPS::getPositionType(utils::optional<nmea::mode_indicator> mode)
{
    switch (*mode) {
        case nmea::mode_indicator::invalid:
            return GPS_SOLUTION_TYPES::INVALID;
        case nmea::mode_indicator::autonomous:
            return GPS_SOLUTION_TYPES::AUTONOMOUS;
        case nmea::mode_indicator::differential:
            return GPS_SOLUTION_TYPES::DIFFERENTIAL;
        case nmea::mode_indicator::estimated:
            return GPS_SOLUTION_TYPES::INVALID;
        case nmea::mode_indicator::manual_input:
            return GPS_SOLUTION_TYPES::INVALID;
        case nmea::mode_indicator::simulated:
            return GPS_SOLUTION_TYPES::INVALID;
        case nmea::mode_indicator::data_not_valid:
            return GPS_SOLUTION_TYPES::INVALID;
        case nmea::mode_indicator::precise:
            return GPS_SOLUTION_TYPES::AUTONOMOUS;
        default:
            return GPS_SOLUTION_TYPES::INVALID;
    }
}

base::Time GPS::buildRockTime(utils::optional<nmea::time> const& optional_time,
    utils::optional<nmea::date> const& optional_date)
{
    auto date = optional_date.value();
    auto time = optional_time.value();
    return base::Time::fromTimeValues(date.year() + 2000,
        static_cast<int>(date.mon()),
        date.day(),
        time.hour(),
        time.minutes(),
        time.seconds(),
        time.milliseconds(),
        0);
}

Position GPS::getPosition(nmea::rmc const& rmc, nmea::gsa const& gsa)
{
    Position position;
    auto mode_indicator = rmc.get_mode_ind().value();
    position.positionType = getPositionType(mode_indicator);
    if (position.positionType != GPS_SOLUTION_TYPES::INVALID) {
        position.time = buildRockTime(rmc.get_time_utc(), rmc.get_date());
        auto optional_latitude = rmc.get_latitude();
        auto optional_longitude = rmc.get_longitude();
        if (optional_latitude.has_value()) {
            position.latitude = optional_latitude.value();
        }
        if (optional_longitude.has_value()) {
            position.longitude = optional_longitude.value();
        }
    }
    else {
        position.time = base::Time::now();
    }
    position.noOfSatellites = 0;
    for (int i = 0; i < gsa.max_satellite_ids; i++) {
        if (gsa.get_satellite_id(i).has_value()) {
            position.noOfSatellites += 1;
        }
    }
    return position;
}

SolutionQuality GPS::getSolutionQuality(nmea::gsa const& gsa)
{
    SolutionQuality solution_quality;
    solution_quality.time = base::Time::now();
    auto optional_pdop = gsa.get_pdop();
    if (optional_pdop.has_value()) {
        solution_quality.pdop = optional_pdop.value();
    }
    auto optional_hdop = gsa.get_hdop();
    if (optional_hdop.has_value()) {
        solution_quality.hdop = optional_hdop.value();
    }
    auto optional_vdop = gsa.get_vdop();
    if (optional_vdop.has_value()) {
        solution_quality.vdop = optional_vdop.value();
    }
    for (int i = 0; i < gsa.max_satellite_ids; i++) {
        auto satellite_id = gsa.get_satellite_id(i);
        if (satellite_id.has_value()) {
            solution_quality.usedSatellites.push_back(satellite_id.value());
        }
    }
    return solution_quality;
}
