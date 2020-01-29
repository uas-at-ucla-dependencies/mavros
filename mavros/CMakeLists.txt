cmake_minimum_required(VERSION 3.10)
project(mavros)

include(GNUInstallDirs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(libmavconn REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

## System dependencies are found with CMake's conventions

## Find Eigen
find_package(Eigen3 REQUIRED)

## Find GeographicLib
find_package(GeographicLib REQUIRED)

## Check if the datasets are installed
include(CheckGeographicLibDatasets)

include(EnableCXX11)
include(MavrosMavlink)

# detect if sensor_msgs has BatteryState.msg
# http://answers.ros.org/question/223769/how-to-check-that-message-exists-with-catkin-for-conditional-compilation-sensor_msgsbatterystate/
list(FIND sensor_msgs_MESSAGE_FILES "msg/BatteryState.msg" BATTERY_STATE_MSG_IDX)
if(${BATTERY_STATE_MSG_IDX} GREATER -1)
  add_definitions(
    -DHAVE_SENSOR_MSGS_BATTERYSTATE_MSG
  )
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###########
## Build ##
###########

include_directories(
  include
)

add_library(mavros
  src/lib/enum_sensor_orientation.cpp
  src/lib/enum_to_string.cpp
  src/lib/ftf_frame_conversions.cpp
  src/lib/ftf_quaternion_utils.cpp
  src/lib/mavlink_diag.cpp
  src/lib/mavros.cpp
  src/lib/rosconsole_bridge.cpp
  src/lib/uas_data.cpp
  src/lib/uas_stringify.cpp
  src/lib/uas_timesync.cpp
)
ament_target_dependencies(mavros
  GeographicLib EIGEN3
  mavlink libmavconn mavros_msgs
  rclcpp sensor_msgs diagnostic_updater eigen_conversions angles
  diagnostic_msgs geographic_msgs pluginlib std_srvs tf2_eigen tf2_ros
)
target_link_libraries(mavros atomic)

add_library(mavros_plugins
  src/plugins/3dr_radio.cpp
  src/plugins/actuator_control.cpp
  src/plugins/altitude.cpp
  src/plugins/command.cpp
  src/plugins/dummy.cpp
  src/plugins/ftp.cpp
  src/plugins/global_position.cpp
  src/plugins/hil.cpp
  src/plugins/home_position.cpp
  src/plugins/imu.cpp
  src/plugins/local_position.cpp
  src/plugins/manual_control.cpp
  src/plugins/param.cpp
  src/plugins/rc_io.cpp
  # src/plugins/safety_area.cpp
  # src/plugins/setpoint_accel.cpp
  # src/plugins/setpoint_attitude.cpp
  # src/plugins/setpoint_position.cpp
  # src/plugins/setpoint_raw.cpp
  # src/plugins/setpoint_velocity.cpp
  # src/plugins/sys_status.cpp
  # src/plugins/sys_time.cpp
  # src/plugins/vfr_hud.cpp
  src/plugins/waypoint.cpp
  # src/plugins/wind_estimation.cpp
)
target_link_libraries(mavros_plugins PUBLIC mavros)

pluginlib_export_plugin_description_file(mavros "mavros_plugins.xml")

## Declare a cpp executable
add_executable(mavros_node
  src/mavros_node.cpp
)
target_link_libraries(mavros_node PUBLIC mavros)

add_executable(gcs_bridge
  src/gcs_bridge.cpp
)
target_link_libraries(gcs_bridge PUBLIC mavros)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
# catkin_install_python(PROGRAMS
#   ${SCRIPTS}
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS gcs_bridge mavros mavros_node mavros_plugins
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mavros_plugins.xml
  DESTINATION ${CMAKE_INSTALL_DATADIR}
)

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/launch
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(libmavros-frame-conversions-test test/test_frame_conversions.cpp)
  target_link_libraries(libmavros-frame-conversions-test mavros)

  ament_add_gtest(libmavros-sensor-orientation-test test/test_sensor_orientation.cpp)
  target_link_libraries(libmavros-sensor-orientation-test mavros)

  ament_add_gtest(libmavros-quaternion-utils-test test/test_quaternion_utils.cpp)
  target_link_libraries(libmavros-quaternion-utils-test mavros)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()

##################################
## ament specific configuration ##
##################################
# ament_export_interfaces(mavros_plugins gcs_bridge mavros_node)
ament_export_include_directories(include)
ament_export_libraries(mavros)
ament_export_dependencies(diagnostic_msgs diagnostic_updater eigen_conversions geographic_msgs geometry_msgs libmavconn
  mavros_msgs rosidl_default_runtime nav_msgs pluginlib rclcpp sensor_msgs std_msgs tf2_ros
  Eigen3 GeographicLib)
ament_package()

# vim: ts=2 sw=2 et:
