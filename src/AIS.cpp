#include <nmea0183/AIS.hpp>
#include <nmea0183/Exceptions.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/ais/ais.hpp>

using namespace std;
using namespace marnav;
using namespace nmea0183;

AIS::AIS(Driver& driver)
    : mDriver(driver) {
}

unique_ptr<ais::message> AIS::readMessage() {
    while (true) {
        auto sentence = mDriver.readSentence();
        auto msg = processSentence(*sentence);
        if (msg) {
            return msg;
        }
    }
}

uint32_t AIS::getDiscardedSentenceCount() const {
    return mDiscardedSentenceCount;
}

unique_ptr<ais::message> AIS::processSentence(nmea::sentence const& sentence) {
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

    payloads.push_back(make_pair(
        vdm->get_payload(), vdm->get_n_fill_bits()
    ));

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

template<typename T>
T optionalFloatToRock(utils::optional<T> opt) {
    return opt ? opt.value() : base::unknown<T>();
}

template<typename T>
base::Angle optionalAngleToRock(utils::optional<T> opt) {
    return opt ? base::Angle::fromDeg(opt.value()) : base::Angle();
}

ais_base::Position AIS::getPosition(ais::message_01 const& message) {
    ais_base::Position position;
    position.time = base::Time::now();
    position.mmsi = message.get_mmsi();
    position.course_over_ground = optionalAngleToRock(message.get_cog());
    position.longitude = optionalAngleToRock(message.get_longitude());
    position.latitude = optionalAngleToRock(message.get_latitude());
    position.status = static_cast<ais_base::NavigationalStatus>(
        message.get_nav_status()
    );
    position.high_accuracy_position = message.get_position_accuracy();
    position.yaw = optionalAngleToRock(message.get_hdg());
    position.speed_over_ground = optionalFloatToRock(message.get_sog());
    position.maneuver_indicator = static_cast<ais_base::ManeuverIndicator>(
        message.get_maneuver_indicator()
    );
    position.raim = message.get_raim();
    position.radio_status = message.get_radio_status();
    position.ensureEnumsValid();
    return position;
}

ais_base::VesselInformation AIS::getVesselInformation(ais::message_05 const& message) {
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
    info.length = message.get_to_bow() + message.get_to_stern();
    info.width = message.get_to_port() + message.get_to_starboard();
    info.draft = static_cast<float>(message.get_draught()) / 10;
    info.ship_type = static_cast<ais_base::ShipType>(
        message.get_shiptype()
    );
    info.epfd_fix = static_cast<ais_base::EPFDFixType>(message.get_epfd_fix());
    info.reference_position = Eigen::Vector3d(
        message.get_to_stern(), message.get_to_starboard(), 0
    );

    info.ensureEnumsValid();
    return info;
}

ais_base::VoyageInformation AIS::getVoyageInformation(ais::message_05 const& message) {
    ais_base::VoyageInformation info;
    info.time = base::Time::now();
    info.mmsi = message.get_mmsi();
    info.imo = message.get_imo_number();
    info.destination = message.get_destination();
    return info;
}
