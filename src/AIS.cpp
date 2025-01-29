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
    : mDriver(driver)
{
}

unique_ptr<ais::message> AIS::readMessage()
{
    while (true) {
        auto sentence = mDriver.readSentence();
        auto msg = processSentence(*sentence);
        if (msg) {
            return msg;
        }
    }
}

uint32_t AIS::getDiscardedSentenceCount() const
{
    return mDiscardedSentenceCount;
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
        mDiscardedSentenceCount += payloads.size();
        payloads.clear();

        // Go on if we're receiving the first fragment of a new message
        if (fragment != 1) {
            mDiscardedSentenceCount++;
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
    auto get_to_stern = message.get_to_stern();
    auto get_to_starboard = message.get_to_starboard();
    float length = message.get_to_bow() + get_to_stern;
    float width = message.get_to_port() + get_to_starboard;
    info.length = length;
    info.width = width;
    info.draft = static_cast<float>(message.get_draught()) / 10;
    info.ship_type = static_cast<ais_base::ShipType>(message.get_shiptype());
    info.epfd_fix = static_cast<ais_base::EPFDFixType>(message.get_epfd_fix());
    info.reference_position =
        Eigen::Vector3d(static_cast<double>(get_to_stern - length / 2.0),
            static_cast<double>(get_to_starboard - width / 2.0),
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

std::pair<Eigen::Quaterniond, ais_base::PositionCorrectionStatus> vesselToWorldOrientation(
    const std::optional<base::Angle>& yaw,
    const std::optional<base::Angle>& course_over_ground,
    double speed_over_ground)
{
    if (yaw.has_value() && !std::isnan(yaw.value().getRad())) {
        return {Eigen::Quaterniond(
                    Eigen::AngleAxisd(yaw.value().getRad(), Eigen::Vector3d::UnitZ())),
            ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_HEADING};
    }
    else if (course_over_ground.has_value() &&
             !std::isnan(course_over_ground.value().getRad()) &&
             speed_over_ground >= MIN_SPEED_FOR_VALID_COURSE) {
        return {Eigen::Quaterniond(Eigen::AngleAxisd(course_over_ground.value().getRad(),
                    Eigen::Vector3d::UnitZ())),
            ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_COURSE};
    }
    else {
        return {Eigen::Quaterniond::Identity(),
            ais_base::PositionCorrectionStatus::POSITION_RAW};
    }
}

base::Vector3d sensorToVesselInWorldPose(base::Vector3d sensor2vessel_in_vessel_pos,
    Eigen::Quaterniond vessel2world_ori)
{
    base::Vector3d sensor2vessel_in_world_pos;
    sensor2vessel_in_world_pos = vessel2world_ori * sensor2vessel_in_vessel_pos;

    return sensor2vessel_in_world_pos;
}

base::samples::RigidBodyState convertGPSToUTM(
    const std::optional<ais_base::Position>& position,
    gps_base::UTMConverter utm_converter)
{
    if (!position.has_value()) {
        std::string error_msg = "Position data is unavailable";
        LOG_ERROR_S << error_msg;
        throw std::invalid_argument(error_msg);
    }

    auto position_value = position.value();
    gps_base::Solution sensor2world_solution;
    sensor2world_solution.latitude = position_value.latitude.getDeg();
    sensor2world_solution.longitude = position_value.longitude.getDeg();

    base::samples::RigidBodyState sensor2world_UTM;
    sensor2world_UTM.position =
        utm_converter.convertToUTM(sensor2world_solution).position;
    sensor2world_UTM.time = base::Time::now();

    return sensor2world_UTM;
}

std::pair<base::Angle, base::Angle> convertUTMToGPSInWorldFrame(
    base::samples::RigidBodyState sensor2world_UTM,
    base::Vector3d sensor2vessel_in_world_pos,
    gps_base::UTMConverter utm_converter)
{
    base::samples::RigidBodyState vessel2world_UTM;
    vessel2world_UTM.position = sensor2world_UTM.position + sensor2vessel_in_world_pos;

    gps_base::Solution vessel2world_GPS;
    vessel2world_GPS = utm_converter.convertUTMToGPS(vessel2world_UTM);

    return {base::Angle::fromDeg(vessel2world_GPS.latitude),
        base::Angle::fromDeg(vessel2world_GPS.longitude)};
}

ais_base::Position AIS::applyPositionCorrection(ais_base::Position const& position,
    base::Vector3d const& sensor2vessel_in_vessel_pos,
    gps_base::UTMConverter utm_converter)
{
    if (std::isnan(position.yaw.getRad()) &&
        std::isnan(position.course_over_ground.getRad())) {
        LOG_ERROR_S << "Position can't be corrected because both 'yaw' "
                       "and 'course_over_ground' values are missing."
                    << std::endl;
        return position;
    }

    auto [vessel2world_ori, status] = vesselToWorldOrientation(position.yaw,
        position.course_over_ground,
        position.speed_over_ground);

    if (status == ais_base::PositionCorrectionStatus::POSITION_RAW) {
        LOG_ERROR_S << "Position can't be corrected because 'yaw' value is missing and "
                       "'speed_over_ground' is below the threshold."
                    << std::endl;
        return position;
    }

    auto sensor2vessel_in_world_pos =
        sensorToVesselInWorldPose(sensor2vessel_in_vessel_pos, vessel2world_ori);

    auto [latitude, longitude] =
        convertUTMToGPSInWorldFrame(convertGPSToUTM(position, utm_converter),
            sensor2vessel_in_world_pos,
            utm_converter);

    ais_base::Position corrected_position;
    corrected_position.latitude = latitude;
    corrected_position.longitude = longitude;
    corrected_position.correction_status = status;

    return corrected_position;
}
