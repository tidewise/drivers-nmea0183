rock_library(nmea0183
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(nmea0183_bin Main.cpp
    DEPS nmea0183)

