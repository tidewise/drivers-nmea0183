#include <iostream>
#include <marnav/ais/name.hpp>
#include <nmea0183/Driver.hpp>
#include <nmea0183/AIS.hpp>

using namespace std;
using namespace nmea0183;
using namespace marnav;

void usage(ostream& out) {
    out << "nmea0183_ctl URI CMD\n"
        << "where CMD is:\n"
        << "  log-sentences: continuously shows timestamp and "
           "type of sentences received\n"
        << "  log-ais: continuously shows timestamp and type of AIS messages received\n"
        << std::flush;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        usage(cerr);
        return 1;
    }

    string uri(argv[1]);
    string cmd(argv[2]);

    Driver driver;
    driver.openURI(uri);
    driver.setReadTimeout(base::Time::fromSeconds(3600));

    if (cmd == "log-sentences") {
        while (true) {
            auto sentence = driver.readSentence();
            cout << base::Time::now() << " " << sentence->tag() << std::endl;
        }
    }
    else if (cmd == "log-ais") {
        AIS ais(driver);
        while (true) {
            auto message = ais.readMessage();
            cout << base::Time::now() << " " << ais::to_name(message->type()) << std::endl;
        }
    }

}
