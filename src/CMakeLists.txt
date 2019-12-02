find_package(marnav)
rock_library(nmea0183
    SOURCES Driver.cpp AIS.cpp
    HEADERS Driver.hpp AIS.hpp
    DEPS_PKGCONFIG iodrivers_base ais_base)
target_link_libraries(nmea0183 marnav::marnav)

rock_executable(nmea0183_ctl Main.cpp
    DEPS nmea0183)
