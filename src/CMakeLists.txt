find_package(marnav)

get_target_property(marnav_IMPORTED_CONFIGURATIONS marnav::marnav IMPORTED_CONFIGURATIONS)
get_target_property(marnav_INCLUDE_DIRECTORIES marnav::marnav INTERFACE_INCLUDE_DIRECTORIES)
list(GET marnav_IMPORTED_CONFIGURATIONS 0 marnav_CONFIGURATION)
get_target_property(marnav_LIBRARY marnav::marnav IMPORTED_LOCATION_${marnav_CONFIGURATION})

foreach(marnav_INC ${marnav_INCLUDE_DIRECTORIES})
    set(marnav_CFLAGS "${marnav_CFLAGS} -I${marnav_INC}")
endforeach()

rock_library(nmea0183
    SOURCES Driver.cpp AIS.cpp
    HEADERS Driver.hpp AIS.hpp Exceptions.hpp
    DEPS_PKGCONFIG iodrivers_base ais_base)
target_link_libraries(nmea0183 marnav::marnav)

rock_executable(nmea0183_ctl Main.cpp
    DEPS nmea0183)
