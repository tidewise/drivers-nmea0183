#include <gtest/gtest.h>
#include <nmea0183/Driver.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace nmea0183;

struct DriverTest : public ::testing::Test,
                    public iodrivers_base::Fixture<Driver> {

    void pushCharToDriver(uint8_t c) {
        pushDataToDriver(&c, &c + 1);
    }

    void pushStringToDriver(std::string const& msg) {
        uint8_t const* msg_u8 = reinterpret_cast<uint8_t const*>(msg.c_str());
        pushDataToDriver(msg_u8, msg_u8 + msg.size());
    }
};

TEST_F(DriverTest, it_accepts_a_valid_NMEA_sentence) {
    string msg = "$GPAPB,A,A,0.10,R,N,V,V,11.0,M,DEST,11.0,M,11.0,M*12\r\n";
    pushStringToDriver(msg);
    auto sentence = driver.readSentence();
    ASSERT_TRUE(sentence);
    ASSERT_EQ("APB", sentence->tag());
}

TEST_F(DriverTest, it_handles_partial_messages) {
    string msg = "$GPAPB,A,A,0.10,R,N,V,V,11.0,M,DEST,11.0,M,11.0,M*12\r\n";
    for (size_t i = 0; i < msg.size() - 1; ++i) {
        pushCharToDriver(msg[i]);
        ASSERT_THROW(driver.readSentence(), iodrivers_base::TimeoutError);
    }
    pushCharToDriver('\n');
    auto sentence = driver.readSentence();
    ASSERT_TRUE(sentence);
    ASSERT_EQ("APB", sentence->tag());
}

TEST_F(DriverTest, it_rejects_an_NMEA_sentence_whose_checksum_is_invalid) {
    string msg = "$GPAPB,A,A,0.10,R,N,V,V,11.0,M,DEST,11.0,M,11.0,M*11\r\n";
    pushStringToDriver(msg);
    ASSERT_THROW(driver.readSentence(), iodrivers_base::TimeoutError);
}

TEST_F(DriverTest, it_skips_garbage) {
    string msg = "$GPAPB,A,A,0,M,11.0,M*12\r\nsomestuff$eoijroeirj\r"
                 "$GPAPB,A,A,0.10,R,N,V,V,11.0,M,DEST,11.0,M,11.0,M*12\r\n";
    pushStringToDriver(msg);
    auto sentence = driver.readSentence();
    ASSERT_TRUE(sentence);
    ASSERT_EQ("APB", sentence->tag());
}

TEST_F(DriverTest, it_skips_a_message_start_if_the_message_is_bigger_than_NMEA_max_sentence_length) {
    // Need to feed more bytes than the driver's internal buffer. Without the
    // sentence length protection, iodrivers_Base will complain
    string msg = "$GPAPB,A,A,0,M,11.0,M*12somestuff$eoijroeirjabcdea"
                 "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabcdea"
                 "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabcdea"
                 "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabcdea"
                 "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaabcdea"
                 "$GPAPB,A,A,0.10,R,N,V,V,11.0,M,DEST,11.0,M,11.0,M*12\r\n";
    pushStringToDriver(msg);
    auto sentence = driver.readSentence();
    ASSERT_TRUE(sentence);
    ASSERT_EQ("APB", sentence->tag());
}
