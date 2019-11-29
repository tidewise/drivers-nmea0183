find_package(marnav)
rock_library(nmea0183
    SOURCES Driver.cpp
    HEADERS Driver.hpp
    DEPS_PKGCONFIG iodrivers_base)
target_link_libraries(nmea0183 marnav::marnav)
