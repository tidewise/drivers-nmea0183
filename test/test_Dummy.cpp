#include <boost/test/unit_test.hpp>
#include <nmea0183/Dummy.hpp>

using namespace nmea0183;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    nmea0183::DummyClass dummy;
    dummy.welcome();
}
