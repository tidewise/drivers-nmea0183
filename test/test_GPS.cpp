#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <marnav/nmea/gsa.hpp>
#include <marnav/nmea/rmc.hpp>
#include <nmea0183/GPS.hpp>

using namespace marnav;
using namespace std;
using namespace nmea0183;

struct GPSTest : public ::testing::Test, public iodrivers_base::Fixture<Driver> {

    GPS gps;

    GPSTest()
        : gps(driver)
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
};

const std::string rmc_string =
    "$GNRMC,000848.00,V,2253.8645,S,04312.0880,W,,,060180,,,N*51\r\n";
const std::string gsa_string = "$GNGSA,A,1,,,,,,,,,,,,,2.0,1.7,1.0*2B\r\n";

TEST_F(GPSTest, it_gets_a_gps_position_from_a_real_and_valid_rmc_and_gsa_messages)
{
    pushStringToDriver(rmc_string);
    auto rmc_sentence = driver.readSentence();
    auto rmc = nmea::sentence_cast<nmea::rmc>(rmc_sentence);
    pushStringToDriver(gsa_string);
    auto gsa_sentence = driver.readSentence();
    auto gsa = nmea::sentence_cast<nmea::gsa>(gsa_sentence);
    auto gps_position = gps.getPosition(*rmc, *gsa);
    ASSERT_NEAR(gps_position.latitude, -22.8977, 1e-3);
    ASSERT_NEAR(gps_position.longitude, -43.2015, 1e-3);
    ASSERT_EQ(gps_position.noOfSatellites, 0);
    ASSERT_EQ(gps_position.positionType, gps_base::GPS_SOLUTION_TYPES::NO_SOLUTION);
}

TEST_F(GPSTest, it_gets_a_gps_solution_quality_from_a_real_and_valid_gsa_messages)
{
    pushStringToDriver(gsa_string);
    auto gsa_sentence = driver.readSentence();
    auto gsa = nmea::sentence_cast<nmea::gsa>(gsa_sentence);
    auto solution_quality = gps.getSolutionQuality(*gsa);
    ASSERT_NEAR(solution_quality.hdop, 1.7, 1e-3);
    ASSERT_NEAR(solution_quality.pdop, 2.0, 1e-3);
    ASSERT_NEAR(solution_quality.vdop, 1.0, 1e-3);
}

TEST_F(GPSTest, it_converts_rmc_with_a_gsa_message_into_gps_position)
{
    marnav::nmea::rmc rmc;
    rmc.set_lat(geo::latitude{12.34});
    rmc.set_lon(geo::longitude{10.12});
    rmc.set_mode_indicator(nmea::mode_indicator::autonomous);
    marnav::nmea::gsa gsa;
    gsa.set_satellite_id(0, 55);
    gsa.set_satellite_id(1, 155);
    auto gps_position = gps.getPosition(rmc, gsa);
    ASSERT_NEAR(gps_position.latitude, 12.34, 1e-3);
    ASSERT_NEAR(gps_position.longitude, 10.12, 1e-3);
    ASSERT_EQ(gps_position.noOfSatellites, 2);
    ASSERT_EQ(gps_position.positionType, gps_base::GPS_SOLUTION_TYPES::AUTONOMOUS_2D);
}

TEST_F(GPSTest, it_converts_a_gsa_message_into_gps_solution_quality)
{
    marnav::nmea::gsa gsa;
    gsa.set_satellite_id(0, 55);
    gsa.set_satellite_id(1, 155);
    gsa.set_hdop(1.1);
    gsa.set_pdop(2.2);
    gsa.set_vdop(3.3);
    auto solution_quality = gps.getSolutionQuality(gsa);
    ASSERT_NEAR(solution_quality.hdop, 1.1, 1e-3);
    ASSERT_NEAR(solution_quality.pdop, 2.2, 1e-3);
    ASSERT_NEAR(solution_quality.vdop, 3.3, 1e-3);
}

