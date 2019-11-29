# nmea0183

Device driver to parse a NMEA0183 stream

This library does NOT implement the protocol parsing itself.
We're using 'marnav' for this. It is instead only proposing
a iodrivers_base-based class to read NMEA streams in general,
and/or an AIS stream in particular (since AIS streams are
a bit strange)

## Usage: NMEA Sentences

`nmea0183::Driver` allows reading NMEA sentences out of any URI `iodrivers_base`
supports.

~~~ cpp
using namespace nmea0183;

Driver driver;
driver.openURI("serial:///dev/ttyUSB0:115200");
driver.setReadTimeout(base::Time::fromSeconds(10));

while(true) {
    // Sentence is a unique_ptr<marnav::nmea::sentence>
    auto sentence = driver.readSentence();
}
~~~

See [marnav's documentation](https://github.com/mariokonrad/marnav) to see what
you can do with marnav itself.

## Usage: AIS Messages

AIS messages on NMEA0183 are made up of multiple NMEA sentences. The `AIS` class
does the reassembly. The assumption is that the NMEA sentences that form the
same AIS message should arrive in a short period of time.

~~~ cpp
using namespace nmea0183;

Driver ais;
ais.openURI("serial:///dev/ttyUSB0:115200");
ais.setReadTimeout(base::Time::fromSeconds(10));

while(true) {
    // Sentence is a unique_ptr<marnav::ais::message>
    auto message = ais.readMessage();
}
~~~

See [marnav's documentation](https://github.com/mariokonrad/marnav) to see what
you can do with marnav itself.

# License

LGPLv2 or later

# Installation

The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/stable/documentation/installation.html)
on how to install Rock.

However, if you feel that it's too heavy for your needs, Rock aims at having
most of its "library" packages (such as this one) to follow best practices. See
[this page](http://rock-robotics.org/stable/documentation/packages/outside_of_rock.html)
for installation instructions outside of Rock.
