#include <nmea0183/AIS.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/ais/ais.hpp>

using namespace marnav;
using namespace nmea0183;

AIS::AIS(Driver& driver)
    : m_driver(driver) {
}

std::unique_ptr<ais::message> AIS::readMessage() {
    std::vector<std::pair<std::string, uint32_t>> payloads;

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