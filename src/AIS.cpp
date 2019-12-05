#include <nmea0183/AIS.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/ais/ais.hpp>

using namespace std;
using namespace marnav;
using namespace nmea0183;

AIS::AIS(Driver& driver)
    : m_driver(driver) {
}

unique_ptr<ais::message> AIS::readMessage() {
    vector<pair<string, uint32_t>> payloads;

    while (true) {
        auto sentence = m_driver.readSentence();
        if (sentence->id() != nmea::sentence_id::VDM) {
            continue;
        }

        auto vdm = nmea::sentence_cast<nmea::vdm>(sentence);

        size_t n_fragments = vdm->get_n_fragments();
        size_t fragment = vdm->get_fragment();
        if (fragment != payloads.size() + 1) {
            payloads.clear();
            if (fragment != 1) {
                continue;
            }
        }

        payloads.push_back(make_pair(
            vdm->get_payload(), vdm->get_n_fill_bits()
        ));

        if (payloads.size() == n_fragments) {
            auto message = ais::make_message(payloads);
            payloads.clear();
            return message;
        }
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
    info.name = message.get_shipname();
    info.call_sign = message.get_callsign();
    info.length = message.get_to_bow() + message.get_to_stern();
    info.width = message.get_to_port() + message.get_to_starboard();
    info.draft = message.get_draught();
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
