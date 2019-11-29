#ifndef NMEA0183_DRIVER_HPP
#define NMEA0183_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <marnav/nmea/nmea.hpp>
#include <marnav/nmea/sentence.hpp>

namespace nmea0183 {
    /**
     * Driver that extracts NMEA0183 sentences
     */
    class Driver : public iodrivers_base::Driver {
        static const int MAX_SENTENCE_LENGTH = marnav::nmea::sentence::max_length;
        static const int BUFFER_SIZE = MAX_SENTENCE_LENGTH;

    protected:
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

    public:
        Driver();

        std::unique_ptr<marnav::nmea::sentence> readSentence();
    };
}

#endif
