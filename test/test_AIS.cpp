#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <nmea0183/AIS.hpp>

using namespace nmea0183;

struct AISTest : public ::testing::Test,
                 public iodrivers_base::Fixture<Driver> {

    AIS ais;

    AISTest()
        : ais(driver) {
    }

    void pushCharToDriver(uint8_t c) {
        pushDataToDriver(&c, &c + 1);
    }

    void pushStringToDriver(std::string const& msg) {
        uint8_t const* msg_u8 = reinterpret_cast<uint8_t const*>(msg.c_str());
        pushDataToDriver(msg_u8, msg_u8 + msg.size());
    }
};

const std::vector<std::string> ais_strings = {
    "!AIVDM,2,1,3,B,55P5TL01VIaAL@7WKO@mBplU@<PDhh000000001S;AJ::4A80?4i@E53,0*3E\r\n",
    "!AIVDM,2,2,3,B,1@0000000000000,2*55\r\n"
};

TEST_F(AISTest, it_reassembles_AIS_messages) {
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
}

TEST_F(AISTest, it_skips_sentences_that_do_not_follow_each_other) {
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
}

TEST_F(AISTest, it_drops_a_sentence_that_do_not_start_a_multisentence_message) {
    pushStringToDriver(ais_strings[1]);
    pushStringToDriver(ais_strings[0]);
    pushStringToDriver(ais_strings[1]);

    auto msg = ais.readMessage();
    ASSERT_EQ(marnav::ais::message_id::static_and_voyage_related_data, msg->type());
}