#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <marnav/ais/message_01.hpp>
#include <nmea0183/AIS.hpp>

using namespace marnav;
using namespace nmea0183;

struct AISTest : public ::testing::Test, public iodrivers_base::Fixture<Driver> {

    AIS ais;

    AISTest()
        : ais(driver)
    {
    }

    void pushCharToDriver(uint8_t c)
    {
        pushDataToDriver(&c, &c + 1);
    }

    void pushStringToDriver(std::string const& msg)
    {
        uint8_t const* msg_u8 = reinterpret_cast<uint8_t const*>(msg.c_str());
        pushDataToDriver(msg_u8, msg_u8 + msg.size());
    }

    gps_base::UTMConverter createUTMConverter()
    {
        gps_base::UTMConversionParameters parameters = {Eigen::Vector3d(1, 1, 0),
            11,
            true};
        gps_base::UTMConverter converter(parameters);

        return converter;
    }
};

const std::vector<std::string> ais_strings = {
    "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E\r\n",
    "!AIVDM,2,2,3,B,1@0000000000000,2*55\r\n"};

const std::vector<std::string> invalid_ais_strings = {
    "$AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@BplU@<PDhh000000001S;AJ::4A80?4i@E53,0*53\r\n",
    "!AIVDM,2,2,3,B,1@0000000000000,2*55\r\n"};

TEST_F(AISTest, it_reassembles_AIS_messages)
{
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
    ASSERT_EQ(0, ais.getDiscardedSentenceCount());
}

TEST_F(AISTest, it_throws_MarnavParsingError_if_the_embedded_message_is_invalid)
{
    pushStringToDriver(invalid_ais_strings[0]);
    pushStringToDriver(invalid_ais_strings[1]);

    // Separate reading sentences and message, to make sure that the messages
    // were indeed valid from the p.o.v. of sentence formatting
    auto sentence0 = driver.readSentence();
    auto sentence1 = driver.readSentence();
    ais.processSentence(*sentence0);
    ASSERT_THROW(ais.processSentence(*sentence1), MarnavParsingError);
}

TEST_F(AISTest, it_skips_sentences_that_do_not_follow_each_other)
{
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
    ASSERT_EQ(1, ais.getDiscardedSentenceCount());
}

TEST_F(AISTest, it_drops_a_sentence_that_do_not_start_a_multisentence_message)
{
    pushStringToDriver(ais_strings[1]);
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
    ASSERT_EQ(1, ais.getDiscardedSentenceCount());
}

TEST_F(AISTest, it_converts_marnav_message01_into_a_Position)
{
    ais::message_01 msg;
    msg.set_mmsi(utils::mmsi(1234567));
    msg.set_nav_status(ais::navigation_status::at_anchor);
    msg.set_rot(ais::rate_of_turn(10.0));
    msg.set_sog(10);
    msg.set_position_accuracy(true);
    msg.set_cog(15);
    msg.set_hdg(25);
    msg.set_timestamp(22);
    msg.set_maneuver_indicator(ais::maneuver_indicator_id::normal);
    msg.set_raim(true);
    msg.set_radio_status(1234);

    auto position = AIS::getPosition(msg);
    ASSERT_EQ(1234567, position.mmsi);
    ASSERT_EQ(0, position.imo);
    ASSERT_EQ(ais_base::STATUS_AT_ANCHOR, position.status);
    ASSERT_TRUE(base::isUnknown(position.yaw_velocity)); // not converted
    ASSERT_FLOAT_EQ(5.14444, position.speed_over_ground);
    ASSERT_TRUE(position.high_accuracy_position);
    ASSERT_FLOAT_EQ(-15, position.course_over_ground.getDeg());
    ASSERT_FLOAT_EQ(-25, position.yaw.getDeg());
    ASSERT_EQ(ais_base::MANEUVER_NORMAL, position.maneuver_indicator);
    ASSERT_TRUE(position.raim);
    ASSERT_EQ(1234, position.radio_status);
}

TEST_F(AISTest, it_sets_STATUS_NOT_DEFINED_for_status_lower_than_STATUS_MIN)
{
    ais::message_01 msg;
    msg.set_nav_status(static_cast<ais::navigation_status>(ais_base::STATUS_MIN - 1));
    auto position = AIS::getPosition(msg);
    ASSERT_EQ(ais_base::STATUS_NOT_DEFINED, position.status);
}

TEST_F(AISTest, it_sets_STATUS_NOT_DEFINED_for_status_higher_than_STATUS_MAX)
{
    ais::message_01 msg;
    msg.set_nav_status(static_cast<ais::navigation_status>(ais_base::STATUS_MAX + 1));
    auto position = AIS::getPosition(msg);
    ASSERT_EQ(ais_base::STATUS_NOT_DEFINED, position.status);
}

TEST_F(AISTest, it_sets_MANEUVER_NOT_AVAILABLE_for_status_lower_than_MANEUVER_MIN)
{
    ais::message_01 msg;
    msg.set_maneuver_indicator(
        static_cast<ais::maneuver_indicator_id>(ais_base::MANEUVER_MIN - 1));
    auto position = AIS::getPosition(msg);
    ASSERT_EQ(ais_base::MANEUVER_NOT_AVAILABLE, position.maneuver_indicator);
}

TEST_F(AISTest, it_sets_MANEUVER_NOT_AVAILABLE_for_status_higher_than_MANEUVER_MAX)
{
    ais::message_01 msg;
    msg.set_maneuver_indicator(
        static_cast<ais::maneuver_indicator_id>(ais_base::MANEUVER_MAX + 1));
    auto position = AIS::getPosition(msg);
    ASSERT_EQ(ais_base::MANEUVER_NOT_AVAILABLE, position.maneuver_indicator);
}

TEST_F(AISTest, it_leaves_absent_optional_fields_as_NaN_in_Position)
{
    ais::message_01 msg;
    auto position = AIS::getPosition(msg);

    ASSERT_TRUE(base::isUnknown(position.course_over_ground));
    ASSERT_TRUE(base::isUnknown(position.latitude));
    ASSERT_TRUE(base::isUnknown(position.longitude));
    ASSERT_TRUE(base::isUnknown(position.yaw));
    ASSERT_TRUE(base::isUnknown(position.yaw_velocity)); // not converted
    ASSERT_TRUE(base::isUnknown(position.speed_over_ground));
}

TEST_F(AISTest, it_converts_marnav_message05_into_a_VesselInformation)
{
    ais::message_05 msg;
    msg.set_mmsi(utils::mmsi(123456));
    msg.set_imo_number(7890);
    msg.set_callsign("CALL");
    msg.set_shipname("NAME");
    msg.set_shiptype(ais::ship_type::cargo);
    msg.set_to_bow(5);
    msg.set_to_stern(10);
    msg.set_to_port(2);
    msg.set_to_starboard(4);
    msg.set_epfd_fix(ais::epfd_fix_type::combined_gps_glonass);
    msg.set_draught(7);
    msg.set_destination("DEST");

    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ(123456, info.mmsi);
    ASSERT_EQ(7890, info.imo);
    ASSERT_EQ("CALL", info.call_sign);
    ASSERT_EQ("NAME", info.name);
    ASSERT_EQ(ais_base::SHIP_TYPE_CARGO, info.ship_type);
    ASSERT_EQ(15, info.length);
    ASSERT_EQ(6, info.width);
    ASSERT_EQ(base::Vector3d(2.5, 1, 0), info.reference_position);
    ASSERT_EQ(ais_base::EPFD_COMBINED_GPS_GLONASS, info.epfd_fix);
    ASSERT_NEAR(0.7, info.draft, 1e-2);
}

TEST_F(AISTest, it_removes_trailing_spaces_in_the_name)
{
    ais::message_05 msg;
    msg.set_shipname("NAME with SPACES   ");
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ("NAME with SPACES", info.name);
}

TEST_F(AISTest, it_removes_trailing_spaces_in_the_callsign)
{
    ais::message_05 msg;
    msg.set_callsign("CALL    ");
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ("CALL", info.call_sign);
}

TEST_F(AISTest, it_sets_SHIP_TYPE_NOT_AVAILABLE_for_ship_types_lower_than_MIN)
{
    ais::message_05 msg;
    msg.set_shiptype(static_cast<ais::ship_type>(ais_base::SHIP_TYPE_MIN - 1));
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ(ais_base::SHIP_TYPE_NOT_AVAILABLE, info.ship_type);
}

TEST_F(AISTest, it_sets_SHIP_TYPE_NOT_AVAILABLE_for_ship_types_higher_than_MAX)
{
    ais::message_05 msg;
    msg.set_shiptype(static_cast<ais::ship_type>(ais_base::SHIP_TYPE_MAX + 1));
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ(ais_base::SHIP_TYPE_NOT_AVAILABLE, info.ship_type);
}

TEST_F(AISTest, it_sets_EPFD_UNDEFINED_for_epfd_fix_lower_than_MIN)
{
    ais::message_05 msg;
    msg.set_epfd_fix(static_cast<ais::epfd_fix_type>(ais_base::EPFD_MIN - 1));
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ(ais_base::EPFD_UNDEFINED, info.epfd_fix);
}

TEST_F(AISTest, it_sets_EPFD_UNDEFINED_for_epfd_fix_higher_than_MAX)
{
    ais::message_05 msg;
    msg.set_epfd_fix(static_cast<ais::epfd_fix_type>(ais_base::EPFD_MAX + 1));
    auto info = AIS::getVesselInformation(msg);
    ASSERT_EQ(ais_base::EPFD_UNDEFINED, info.epfd_fix);
}

TEST_F(AISTest, it_converts_marnav_message05_into_a_VoyageInformation)
{
    ais::message_05 msg;
    msg.set_mmsi(utils::mmsi(123456));
    msg.set_imo_number(7890);
    msg.set_callsign("CALL");
    msg.set_shipname("NAME");
    msg.set_shiptype(ais::ship_type::cargo);
    msg.set_to_bow(5);
    msg.set_to_stern(10);
    msg.set_to_port(2);
    msg.set_to_starboard(4);
    msg.set_epfd_fix(ais::epfd_fix_type::combined_gps_glonass);
    msg.set_draught(7);
    msg.set_destination("DEST");

    auto info = AIS::getVoyageInformation(msg);
    ASSERT_EQ(123456, info.mmsi);
    ASSERT_EQ(7890, info.imo);
    ASSERT_EQ("DEST", info.destination);
}

TEST_F(AISTest, it_selects_yaw_as_vessel_heading_source_if_available)
{
    ais::message_01 msg;
    msg.set_sog(0);
    msg.set_hdg(90);
    auto position = AIS::getPosition(msg);

    auto [ori, status] = AIS::selectVesselHeadingSource(position.yaw,
        position.course_over_ground,
        position.speed_over_ground);

    ASSERT_TRUE(ori.isApprox(
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()))));
    ASSERT_EQ(status,
        ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_HEADING);
}

TEST_F(AISTest,
    it_selects_cog_as_vessel_heading_source_if_no_yaw_and_sog_is_above_threshold_for_cog)
{
    ais::message_01 msg;
    msg.set_sog(0.5);
    msg.set_cog(90);
    auto position = AIS::getPosition(msg);

    auto [ori, status] = AIS::selectVesselHeadingSource(position.yaw,
        position.course_over_ground,
        position.speed_over_ground);

    ASSERT_TRUE(ori.isApprox(
        Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()))));
    ASSERT_EQ(status, ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_COURSE);
}

TEST_F(AISTest,
    it_selects_identity_as_vessel_heading_source_if_no_yaw_and_sog_is_below_threshold_for_cog)
{
    ais::message_01 msg;
    msg.set_sog(0.1);
    msg.set_cog(90);
    auto position = AIS::getPosition(msg);

    auto [ori, status] = AIS::selectVesselHeadingSource(position.yaw,
        position.course_over_ground,
        position.speed_over_ground);

    ASSERT_TRUE(
        ori.isApprox(Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()))));
    ASSERT_EQ(status, ais_base::PositionCorrectionStatus::POSITION_RAW);
}

TEST_F(AISTest, it_selects_identity_as_vessel_heading_source_if_no_yaw_or_cog)
{
    ais::message_01 msg;
    auto position = AIS::getPosition(msg);

    auto [ori, status] = AIS::selectVesselHeadingSource(position.yaw,
        position.course_over_ground,
        position.speed_over_ground);

    ASSERT_TRUE(
        ori.isApprox(Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()))));
    ASSERT_EQ(status, ais_base::PositionCorrectionStatus::POSITION_RAW);
}

TEST_F(AISTest, it_does_no_correction_if_no_yaw_and_sog_is_below_threshold_for_cog)
{
    ais::message_01 msg;
    msg.set_latitude(geo::latitude(45));
    msg.set_longitude(geo::longitude(-120));
    msg.set_sog(0.1);
    msg.set_cog(90);
    auto position = AIS::getPosition(msg);

    base::Vector3d sensor2vessel_pos(-100.0, -50.0, 0.0);
    gps_base::UTMConverter utm_converter = createUTMConverter();

    ais_base::Position corrected_position =
        AIS::applyPositionCorrection(position, sensor2vessel_pos, utm_converter);
    ASSERT_EQ(corrected_position.correction_status,
        ais_base::PositionCorrectionStatus::POSITION_RAW);
    ASSERT_EQ(position.time, corrected_position.time);
}

TEST_F(AISTest, it_does_no_correction_if_no_yaw_or_cog)
{
    ais::message_01 msg;
    msg.set_latitude(geo::latitude(45));
    msg.set_longitude(geo::longitude(-120));
    auto position = AIS::getPosition(msg);

    base::Vector3d sensor2vessel_pos(-100.0, -50.0, 0.0);
    gps_base::UTMConverter utm_converter = createUTMConverter();

    ais_base::Position corrected_position =
        AIS::applyPositionCorrection(position, sensor2vessel_pos, utm_converter);
    ASSERT_EQ(corrected_position.correction_status,
        ais_base::PositionCorrectionStatus::POSITION_RAW);
}

TEST_F(AISTest, it_corrects_position_using_yaw)
{
    auto utm_converter = createUTMConverter();
    base::samples::RigidBodyState vessel2world, sensor2world;
    vessel2world.position = base::Vector3d(10, 10, 0);
    sensor2world.position = base::Vector3d(9, 11, 0);
    auto sensor2world_gps = utm_converter.convertUTMToGPS(sensor2world);

    ais::message_01 msg_position;
    msg_position.set_latitude(geo::latitude(sensor2world_gps.latitude));
    msg_position.set_longitude(geo::longitude(sensor2world_gps.longitude));
    msg_position.set_hdg(90);
    auto position = AIS::getPosition(msg_position);

    ais::message_05 msg_info;
    msg_info.set_to_port(9);
    msg_info.set_to_starboard(11);
    msg_info.set_to_bow(21);
    msg_info.set_to_stern(19);
    auto info = AIS::getVesselInformation(msg_info);

    auto corrected_position =
        AIS::applyPositionCorrection(position, info.reference_position, utm_converter);

    auto vessel_pos = utm_converter.convertUTMToGPS(vessel2world);
    ASSERT_NEAR(vessel_pos.latitude, corrected_position.latitude.getDeg(), 1e-3);
    ASSERT_NEAR(vessel_pos.longitude, corrected_position.longitude.getDeg(), 1e-3);
    ASSERT_EQ(corrected_position.correction_status,
        ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_HEADING);
    ASSERT_EQ(position.time, corrected_position.time);
}

TEST_F(AISTest, it_corrects_position_using_cog)
{
    auto utm_converter = createUTMConverter();
    base::samples::RigidBodyState vessel2world, sensor2world;
    vessel2world.position = base::Vector3d(10, 10, 0);
    sensor2world.position = base::Vector3d(9, 11, 0);
    auto sensor2world_gps = utm_converter.convertUTMToGPS(sensor2world);

    ais::message_01 msg_position;
    msg_position.set_latitude(geo::latitude(sensor2world_gps.latitude));
    msg_position.set_longitude(geo::longitude(sensor2world_gps.longitude));
    msg_position.set_sog(0.5);
    msg_position.set_cog(90);
    auto position = AIS::getPosition(msg_position);

    ais::message_05 msg_info;
    msg_info.set_to_port(9);
    msg_info.set_to_starboard(11);
    msg_info.set_to_bow(21);
    msg_info.set_to_stern(19);
    auto info = AIS::getVesselInformation(msg_info);

    auto corrected_position =
        AIS::applyPositionCorrection(position, info.reference_position, utm_converter);

    auto vessel_pos = utm_converter.convertUTMToGPS(vessel2world);
    ASSERT_NEAR(vessel_pos.latitude, corrected_position.latitude.getDeg(), 1e-3);
    ASSERT_NEAR(vessel_pos.longitude, corrected_position.longitude.getDeg(), 1e-3);
    ASSERT_EQ(corrected_position.correction_status,
        ais_base::PositionCorrectionStatus::POSITION_CENTERED_USING_COURSE);
    ASSERT_EQ(position.time, corrected_position.time);
}

TEST_F(AISTest, it_converts_VesselInformation_into_a_message05)
{
    ais_base::VesselInformation info;
    info.mmsi = 123456;
    info.imo = 7890;
    info.name = "Vingilot";
    info.call_sign = "AAA";
    info.ship_type = ais_base::ShipType::SHIP_TYPE_NOT_AVAILABLE;
    info.epfd_fix = ais_base::EPFDFixType::EPFD_UNDEFINED;
    info.reference_position = {5.0, 2.0, 0};
    info.width = 14.0;
    info.length = 40.0;
    info.draft = 2.0;

    auto message = AIS::getMessageFromVesselInformation(info);

    ASSERT_EQ(message.get_mmsi(), utils::mmsi{123456});
    ASSERT_EQ(message.get_imo_number(), 7890);
    ASSERT_EQ(message.get_shipname(), "Vingilot");
    ASSERT_EQ(message.get_callsign(), "AAA");
    ASSERT_EQ(static_cast<int>(message.get_shiptype()), 0);
    ASSERT_EQ(static_cast<int>(message.get_epfd_fix()), 0);
    ASSERT_EQ(message.get_to_bow(), 15.0);
    ASSERT_EQ(message.get_to_stern(), 25.0);
    ASSERT_EQ(message.get_to_port(), 5.0);
    ASSERT_EQ(message.get_to_starboard(), 9.0);
    ASSERT_EQ(message.get_draught(), 20.0);
}

TEST_F(AISTest, it_converts_Position_into_a_message01)
{
    ais_base::Position position;
    position.mmsi = 123456;
    position.status = ais_base::NavigationalStatus::STATUS_UNDER_WAY_USING_ENGINE;
    position.high_accuracy_position = true;
    position.latitude = base::Angle::fromDeg(45);
    position.longitude = base::Angle::fromDeg(-120);
    position.course_over_ground = base::Angle::fromDeg(90);
    position.yaw = base::Angle::fromDeg(45);
    position.speed_over_ground = 5.0;
    position.maneuver_indicator = ais_base::ManeuverIndicator::MANEUVER_NOT_AVAILABLE;
    position.raim = true;
    position.radio_status = 12345;

    auto message = AIS::getMessageFromPosition(position);

    ASSERT_EQ(message.get_mmsi(), utils::mmsi{123456});
    ASSERT_EQ(static_cast<int>(message.get_nav_status()), 0);
    ASSERT_TRUE(message.get_position_accuracy());

    ASSERT_TRUE(message.get_latitude().has_value());
    ASSERT_TRUE(message.get_longitude().has_value());
    ASSERT_EQ(message.get_latitude().value().get(), 45);
    ASSERT_EQ(message.get_longitude().value().get(), -120);

    ASSERT_TRUE(message.get_cog().has_value());
    ASSERT_EQ(message.get_cog().value(), 90);

    ASSERT_TRUE(message.get_hdg().has_value());
    ASSERT_EQ(message.get_hdg().value(), 45);

    ASSERT_TRUE(message.get_sog().has_value());
    ASSERT_NEAR(message.get_sog().value(), 9.7192, 1e-1);

    ASSERT_EQ(static_cast<int>(message.get_maneuver_indicator()), 0);
    ASSERT_TRUE(message.get_raim());
    ASSERT_EQ(message.get_radio_status(), 12345);
}

TEST_F(AISTest,
    it_handles_unset_values_in_VesselInformation_when_converting_into_message05)
{
    ais_base::VesselInformation info;

    auto message = AIS::getMessageFromVesselInformation(info);

    ASSERT_EQ(message.get_mmsi(), utils::mmsi{0});
    ASSERT_EQ(message.get_imo_number(), 0);
    ASSERT_EQ(message.get_shipname(), "");
    ASSERT_EQ(message.get_callsign(), "");
    ASSERT_EQ(static_cast<int>(message.get_shiptype()), 0);
    ASSERT_EQ(static_cast<int>(message.get_epfd_fix()), 0);
    ASSERT_EQ(message.get_to_bow(), 0);
    ASSERT_EQ(message.get_to_stern(), 0);
    ASSERT_EQ(message.get_to_port(), 0);
    ASSERT_EQ(message.get_to_starboard(), 0);
    ASSERT_EQ(message.get_draught(), 0.0);
}

TEST_F(AISTest, it_handles_unset_values_in_Position_when_converting_into_message01)
{
    ais_base::Position position;

    auto message = AIS::getMessageFromPosition(position);

    ASSERT_EQ(message.get_mmsi(), utils::mmsi{0});
    ASSERT_EQ(static_cast<int>(message.get_nav_status()), 15);
    ASSERT_FALSE(message.get_position_accuracy());

    ASSERT_FALSE(message.get_latitude().has_value());
    ASSERT_FALSE(message.get_longitude().has_value());

    ASSERT_EQ(message.get_cog().value(), 0);
    ASSERT_EQ(message.get_hdg().value(), 0);
    ASSERT_EQ(message.get_sog().value(), 0);

    ASSERT_EQ(static_cast<int>(message.get_maneuver_indicator()), 0);
    ASSERT_FALSE(message.get_raim());
    ASSERT_EQ(message.get_radio_status(), 0);
}
