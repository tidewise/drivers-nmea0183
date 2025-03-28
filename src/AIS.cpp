#include <base-logging/Logging.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/nmea/vdm.hpp>
#include <nmea0183/AIS.hpp>
#include <nmea0183/Exceptions.hpp>

using namespace std;
using namespace marnav;
using namespace nmea0183;

double constexpr KNOTS_TO_MS = 0.514444;
double constexpr MS_TO_KNOTS = 1.94384;
double constexpr MIN_SPEED_FOR_VALID_COURSE = 0.2;

AIS::AIS(Driver& driver)
    : m_driver(driver)
{
}

unique_ptr<ais::message> AIS::readMessage()
{
    while (true) {
        auto sentence = m_driver.readSentence();
        auto msg = processSentence(*sentence);
        if (msg) {
            return msg;
        }
    }
}

uint32_t AIS::getDiscardedSentenceCount() const
{
    return m_discarded_sentence_count;
}

unique_ptr<ais::message> AIS::processSentence(nmea::sentence const& sentence)
{
    if (sentence.id() != nmea::sentence_id::VDM) {
        return unique_ptr<ais::message>();
    }

    auto vdm = nmea::sentence_cast<nmea::vdm>(&sentence);

    size_t n_fragments = vdm->get_n_fragments();
    size_t fragment = vdm->get_fragment();
    if (fragment != payloads.size() + 1) {
        m_discarded_sentence_count += payloads.size();
        payloads.clear();

        // Go on if we're receiving the first fragment of a new message
        if (fragment != 1) {
            m_discarded_sentence_count++;
            return unique_ptr<ais::message>();
        }
    }

    payloads.push_back(make_pair(vdm->get_payload(), vdm->get_n_fill_bits()));

    if (payloads.size() != n_fragments) {
        return unique_ptr<ais::message>();
    }

    try {
        auto payloads = std::move(this->payloads);
        return ais::make_message(payloads);
    }
    catch (std::exception const& e) {
        throw MarnavParsingError(e.what());
    }
}

template <typename T> T optionalFloatToRock(utils::optional<T> opt)
{
    return opt ? opt.value() : base::unknown<T>();
}

template <typename T> base::Angle optionalAngleToRock(utils::optional<T> opt)
{
    return opt ? base::Angle::fromDeg(opt.value()) : base::Angle();
}

ais_base::Position AIS::getPosition(ais::message_01 const& message)
{
    ais_base::Position position;
    position.time = base::Time::now();
    position.mmsi = message.get_mmsi();
    position.course_over_ground = optionalAngleToRock(message.get_cog()) * -1;
    position.longitude = optionalAngleToRock(message.get_longitude());
    position.latitude = optionalAngleToRock(message.get_latitude());
    position.status = static_cast<ais_base::NavigationalStatus>(message.get_nav_status());
    position.high_accuracy_position = message.get_position_accuracy();
    position.yaw = optionalAngleToRock(message.get_hdg()) * -1;
    position.speed_over_ground = optionalFloatToRock(message.get_sog()) * KNOTS_TO_MS;
    position.maneuver_indicator =
        static_cast<ais_base::ManeuverIndicator>(message.get_maneuver_indicator());
    position.raim = message.get_raim();
    position.radio_status = message.get_radio_status();
    position.ensureEnumsValid();
    return position;
}

ais_base::VesselInformation AIS::getVesselInformation(ais::message_05 const& message)
{
    ais_base::VesselInformation info;
    info.time = base::Time::now();
    info.mmsi = message.get_mmsi();
    info.imo = message.get_imo_number();
    string name = message.get_shipname();
    int first_not_space = name.find_last_not_of(" ");
    info.name = name.substr(0, first_not_space + 1);
    string call_sign = message.get_callsign();
    first_not_space = call_sign.find_last_not_of(" ");
    info.call_sign = call_sign.substr(0, first_not_space + 1);
    auto distance_to_stern = message.get_to_stern();
    auto distance_to_starboard = message.get_to_starboard();
    float length = message.get_to_bow() + distance_to_stern;
    float width = message.get_to_port() + distance_to_starboard;
    info.length = length;
    info.width = width;
    info.draft = static_cast<float>(message.get_draught()) / 10;
    info.ship_type = static_cast<ais_base::ShipType>(message.get_shiptype());
    info.epfd_fix = static_cast<ais_base::EPFDFixType>(message.get_epfd_fix());
    info.reference_position = Eigen::Vector3d(distance_to_stern - (length / 2.0),
        distance_to_starboard - (width / 2.0),
        0);

    info.ensureEnumsValid();
    return info;
}

ais_base::VoyageInformation AIS::getVoyageInformation(ais::message_05 const& message)
{
    ais_base::VoyageInformation info;
    info.time = base::Time::now();
    info.mmsi = message.get_mmsi();
    info.imo = message.get_imo_number();
    info.destination = message.get_destination();
    return info;
}

std::pair<Eigen::Quaterniond, ais_base::PositionCorrectionStatus> AIS::
    selectVesselHeadingSource(base::Angle const& yaw,
        base::Angle const& course_over_ground,
        double speed_over_ground)
{
    if (!std::isnan(yaw.getRad())) {
        return {
            Eigen::Quaterniond(Eigen::AngleAxisd(yaw.getRad(), Eigen::Vector3d::UnitZ())),
            ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_HEADING};
    }
    else if (!std::isnan(course_over_ground.getRad()) &&
             speed_over_ground >= MIN_SPEED_FOR_VALID_COURSE) {
        return {Eigen::Quaterniond(Eigen::AngleAxisd(course_over_ground.getRad(),
                    Eigen::Vector3d::UnitZ())),
            ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_COURSE};
    }
    else {
        return {Eigen::Quaterniond::Identity(),
            ais_base::PositionCorrectionStatus::POSITION_RAW};
    }
}

base::Vector3d convertGPSToUTM(ais_base::Position const& position,
    gps_base::UTMConverter const& utm_converter)
{
    gps_base::Solution sensor2world_solution;
    sensor2world_solution.latitude = position.latitude.getDeg();
    sensor2world_solution.longitude = position.longitude.getDeg();

    auto sensor2world = utm_converter.convertToUTM(sensor2world_solution);

    return sensor2world.position;
}

base::Vector3d computeVesselPositionInWorldFrame(base::Vector3d const& sensor2vessel_pos,
    Eigen::Quaterniond const& vessel2world_ori,
    base::Vector3d const& sensor2world_pos)
{
    const auto sensor2world_ori = vessel2world_ori;
    base::Vector3d vessel2world_pos;
    vessel2world_pos = sensor2world_pos - (sensor2world_ori * sensor2vessel_pos);
    return vessel2world_pos;
}

std::pair<base::Angle, base::Angle> convertUTMToGPS(
    base::Vector3d const& vessel2world_pos,
    gps_base::UTMConverter const& utm_converter)
{
    base::samples::RigidBodyState vessel2world;
    vessel2world.position = vessel2world_pos;
    gps_base::Solution vessel2world_gps;
    vessel2world_gps = utm_converter.convertUTMToGPS(vessel2world);

    return {base::Angle::fromDeg(vessel2world_gps.latitude),
        base::Angle::fromDeg(vessel2world_gps.longitude)};
}

ais_base::Position AIS::applyPositionCorrection(ais_base::Position const& sensor_pos,
    base::Vector3d const& sensor2vessel_pos,
    gps_base::UTMConverter const& utm_converter)
{
    auto vessel_pos = sensor_pos;
    if (std::isnan(sensor_pos.yaw.getRad()) &&
        std::isnan(sensor_pos.course_over_ground.getRad())) {
        LOG_DEBUG_S << "Position can't be corrected because both 'yaw' "
                       "and 'course_over_ground' values are missing."
                    << std::endl;
        vessel_pos.correction_status = ais_base::PositionCorrectionStatus::POSITION_RAW;
        return sensor_pos;
    }

    auto [vessel2world_ori, status] = selectVesselHeadingSource(sensor_pos.yaw,
        sensor_pos.course_over_ground,
        sensor_pos.speed_over_ground);

    if (status == ais_base::PositionCorrectionStatus::POSITION_RAW) {
        LOG_DEBUG_S << "Position can't be corrected because 'yaw' value is missing and "
                       "'speed_over_ground' is below the threshold."
                    << std::endl;
        vessel_pos.correction_status = status;
        return vessel_pos;
    }

    auto sensor2world_pos = convertGPSToUTM(sensor_pos, utm_converter);

    auto vessel2world_pos = computeVesselPositionInWorldFrame(sensor2vessel_pos,
        vessel2world_ori,
        sensor2world_pos);

    auto [latitude, longitude] = convertUTMToGPS(vessel2world_pos, utm_converter);

    vessel_pos.latitude = latitude;
    vessel_pos.longitude = longitude;
    vessel_pos.correction_status = status;

    return vessel_pos;
}

/**
 * @brief General template for handling values that might be unknown, returning a default
 * value.
 */
template <typename SourceType>
SourceType safe_value(SourceType value, SourceType default_value)
{
    return (base::isUnknown(value)) ? default_value : value;
}

/**
 * @brief General template for handling optional values that might be unknown.
 * It also deals with type casting.
 */
template <typename SourceType, typename TargetType = SourceType>
utils::optional<TargetType> safe_optional(SourceType value)
{
    return (base::isUnknown(value))
               ? utils::optional<TargetType>{}
               : utils::optional<TargetType>{static_cast<TargetType>(value)};
}

/**
 * @brief Method for dealing with latitude and longitude values in base::Angle that might
 * be unknown by returning default values and converting them to the marnav type.
 * If given values exist, they are converted to marnav type
 *
 * @return A pair [marnav::geo::latitude, marnav::geo::longitude]
 */
std::pair<marnav::utils::optional<marnav::geo::latitude>,
    marnav::utils::optional<marnav::geo::longitude>>
safe_optional_gps_position(base::Angle latitude, base::Angle longitude)
{
    return {base::isUnknown(latitude)
                ? marnav::utils::optional<marnav::geo::latitude>{}
                : marnav::utils::optional<marnav::geo::latitude>{latitude.getDeg()},
        base::isUnknown(longitude)
            ? marnav::utils::optional<marnav::geo::longitude>{}
            : marnav::utils::optional<marnav::geo::longitude>{longitude.getDeg()}};
}

ais::message_05 AIS::getMessageFromVesselInformation(
    ais_base::VesselInformation const& info)
{
    ais::message_05 message;

    message.set_mmsi(utils::mmsi{static_cast<unsigned int>(info.mmsi)});
    message.set_imo_number(info.imo);
    message.set_shipname(info.name);
    message.set_callsign(info.call_sign);
    message.set_shiptype(static_cast<marnav::ais::ship_type>(info.ship_type));
    message.set_epfd_fix(static_cast<marnav::ais::epfd_fix_type>(info.epfd_fix));
    message.set_to_bow(safe_value((info.length / 2.0) - info.reference_position.x(), 0.));
    message.set_to_stern(
        safe_value(info.reference_position.x() + (info.length / 2.0), 0.));
    message.set_to_starboard(
        safe_value(info.reference_position.y() + (info.width / 2.0), 0.));
    message.set_to_port(safe_value((info.width / 2.0) - info.reference_position.y(), 0.));
    message.set_draught(safe_value(info.draft * 10.0, 0.));

    return message;
}

ais::message_01 AIS::getMessageFromPosition(ais_base::Position const& position)
{
    ais::message_01 message;

    message.set_mmsi(utils::mmsi{static_cast<unsigned int>(position.mmsi)});
    message.set_nav_status(static_cast<marnav::ais::navigation_status>(position.status));
    message.set_position_accuracy(position.high_accuracy_position);
    auto [safe_latitude, safe_longitude] =
        safe_optional_gps_position(position.latitude, position.longitude);
    message.set_latitude(safe_latitude);
    message.set_longitude(safe_longitude);
    message.set_cog(safe_optional(position.course_over_ground.getDeg()));
    message.set_hdg(safe_optional<double, uint32_t>(position.yaw.getDeg()));
    message.set_sog(safe_optional(position.speed_over_ground * MS_TO_KNOTS));
    message.set_maneuver_indicator(
        static_cast<marnav::ais::maneuver_indicator_id>(position.maneuver_indicator));
    message.set_raim(position.raim);
    message.set_radio_status(position.radio_status);

    return message;
}
