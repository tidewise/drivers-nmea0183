#include <nmea0183/Driver.hpp>

#include <marnav/nmea/checksum.hpp>

using namespace std;
using namespace nmea0183;

Driver::Driver()
    : iodrivers_base::Driver(BUFFER_SIZE) {
}

static uint8_t char2hex(char c) {
    if (c <= '9') {
        return c - '0';
    }
    else if (c <= 'F') {
        return 10 + c - 'A';
    }
    else if (c <= 'f') {
        return 10 + c - 'a';
    }
    throw std::invalid_argument("invalid hex character");
}

int Driver::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    if (buffer[0] != '$' && buffer[0] != '!') {
        return -1;
    }
    else if (buffer_size < 2) {
        return 0;
    }
    else if (buffer_size > MAX_SENTENCE_LENGTH) {
        return -1; // Just eat the '$' and let iodriver_base call us back
    }

    for (size_t i = 1; i < buffer_size; ++i) {
        if (buffer[i - 1] == '\r' && buffer[i] == '\n') {
            uint8_t msg_checksum =
                (char2hex(buffer[i - 3]) << 4) +
                (char2hex(buffer[i - 2]) << 0);

            uint8_t checksum = 0;
            for (size_t j = 1; j < i - 4; ++j) {
                checksum ^= buffer[j];
            }

            if (checksum != msg_checksum) {
                return -1;
            }

            return i + 1;
        }
    }

    return 0;
}

std::unique_ptr<marnav::nmea::sentence> Driver::readSentence() {
    uint8_t buffer[BUFFER_SIZE];
    int sentence_size = readPacket(buffer, BUFFER_SIZE);
    return marnav::nmea::make_sentence(
        std::string(reinterpret_cast<char*>(buffer),
                    reinterpret_cast<char*>(buffer + sentence_size - 2))
    );
}