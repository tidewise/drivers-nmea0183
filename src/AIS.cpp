#include <base-logging/Logging.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/nmea/vdm.hpp>
#include <nmea0183/AIS.hpp>
#include <nmea0183/Exceptions.hpp>

using namespace std;
using namespace marnav;
using namespace nmea0183;

double constexpr KNOTS_TO_MS = 0.514444;
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

base::Vector3d convertPositionInGPSToUTM(ais_base::Position const& position,
    gps_base::UTMConverter const& utm_converter)
{
    gps_base::Solution sensor2world_solution;
    sensor2world_solution.latitude = position.latitude.getDeg();
    sensor2world_solution.longitude = position.longitude.getDeg();

    auto sensor2world = utm_converter.convertToUTM(sensor2world_solution);

    return sensor2world.position;
}

base::Vector3d getVesselPositionInWorldFrame(base::Vector3d const& sensor2vessel_pos,
    Eigen::Quaterniond const& vessel2world_ori)
{
    return -(vessel2world_ori * sensor2vessel_pos);
}

std::pair<base::Angle, base::Angle> convertUTMToGPSInWorldFrame(
    base::Vector3d const& sensor2world_pos,
    base::Vector3d const& vessel2sensor_in_world_pos,
    gps_base::UTMConverter const& utm_converter)
{
    base::samples::RigidBodyState vessel2world;
    vessel2world.position = sensor2world_pos + vessel2sensor_in_world_pos;

    gps_base::Solution vessel2world_gps;
    vessel2world_gps = utm_converter.convertUTMToGPS(vessel2world);

    return {base::Angle::fromDeg(vessel2world_gps.latitude),
        base::Angle::fromDeg(vessel2world_gps.longitude)};
}

ais_base::Position AIS::applyPositionCorrection(ais_base::Position const& position,
    base::Vector3d const& sensor2vessel_pos,
    gps_base::UTMConverter const& utm_converter)
{
    auto sensor2world_gps = position;

    if (std::isnan(sensor2world_gps.yaw.getRad()) &&
        std::isnan(sensor2world_gps.course_over_ground.getRad())) {
        LOG_DEBUG_S << "Position can't be corrected because both 'yaw' "
                       "and 'course_over_ground' values are missing."
                    << std::endl;
        return sensor2world_gps;
    }

    auto [vessel2world_ori, status] = selectVesselHeadingSource(sensor2world_gps.yaw,
        sensor2world_gps.course_over_ground,
        sensor2world_gps.speed_over_ground);

    if (status == ais_base::PositionCorrectionStatus::POSITION_RAW) {
        LOG_DEBUG_S << "Position can't be corrected because 'yaw' value is missing and "
                       "'speed_over_ground' is below the threshold."
                    << std::endl;
        return sensor2world_gps;
    }

    auto sensor2world_pos = convertPositionInGPSToUTM(sensor2world_gps, utm_converter);

    auto vessel2sensor_in_world_pos =
        getVesselPositionInWorldFrame(sensor2vessel_pos, vessel2world_ori);

    auto [latitude, longitude] = convertUTMToGPSInWorldFrame(sensor2world_pos,
        vessel2sensor_in_world_pos,
        utm_converter);

    sensor2world_gps.latitude = latitude;
    sensor2world_gps.longitude = longitude;
    sensor2world_gps.correction_status = status;

    return sensor2world_gps;
}
