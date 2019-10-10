cmake_minimum_required(VERSION 3.10)
project(libmavconn)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)

## System dependencies are found with CMake's conventions
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)
include(MavrosMavlink)
include(GNUInstallDirs)
include(em_expand)

# Fixed in mavlink 2016.7.7
#-> enum values out of int range
#list(APPEND IGNORE_DIALECTS "autoquad")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include
  ${console_bridge_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp
  src/interface.cpp
  src/serial.cpp
  src/tcp.cpp
  src/udp.cpp
)
target_include_directories(mavconn PUBLIC
  ${mavlink_INCLUDE_DIRS}
)
target_link_libraries(mavconn PUBLIC
  Boost::system
  ${console_bridge_LIBRARIES}
)

# Use catkin-supplied em_expand macros to generate source files
em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/mavlink_dialect.h.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h)

em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink_helpers.cpp.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp)

if(DEFINED ENV{CI})
  message(STATUS "CI environment detected. Assume that it is on RO fs")
else()
  message(STATUS "Copy mavlink_dialect.h to source tree")
  file(
    COPY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h
    DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/
    )
endif()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mavconn
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

## Mark cpp header files for installation
install(DIRECTORY include/mavconn/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mavconn
  FILES_MATCHING PATTERN "*.h"
)

## Install cmake files (thanks to cmake_modules package)
install(DIRECTORY cmake/Modules
  DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake
)

##################################
## ament specific configuration ##
##################################
ament_export_include_directories(include)
ament_export_libraries(mavconn)
ament_export_dependencies(Boost console_bridge mavlink)
ament_package(
  CONFIG_EXTRAS ${CMAKE_SOURCE_DIR}/cmake/libmavconn-extras.cmake.in
)

#############
## Testing ##
#############

if(AMENT_ENABLE_TESTING)
  find_package(ament_gtest REQUIRED)
  ament_cmake_gtest(mavconn-test test/test_mavconn.cpp)
  target_link_libraries(mavconn-test mavconn pthread)
endif()

# vim: ts=2 sw=2 et:
