cmake_minimum_required(VERSION 3.10)
project(mavros_extras)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)

include(EnableCXX11)
include(MavrosMavlink)
include(GNUInstallDirs)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###########
## Build ##
###########

add_library(mavros_extras
  src/plugins/adsb.cpp
  src/plugins/cam_imu_sync.cpp
  src/plugins/companion_process_status.cpp
  src/plugins/onboard_computer_status.cpp
  src/plugins/debug_value.cpp
  src/plugins/distance_sensor.cpp
  src/plugins/fake_gps.cpp
  src/plugins/gps_rtk.cpp
  src/plugins/landing_target.cpp
  src/plugins/log_transfer.cpp
  src/plugins/mocap_pose_estimate.cpp
  src/plugins/obstacle_distance.cpp
  src/plugins/odom.cpp
  src/plugins/px4flow.cpp
  src/plugins/rangefinder.cpp
  src/plugins/trajectory.cpp
  src/plugins/vibration.cpp
  src/plugins/vision_pose_estimate.cpp
  src/plugins/vision_speed_estimate.cpp
  src/plugins/wheel_odometry.cpp
  src/plugins/mount_control.cpp
)
ament_target_dependencies(mavros_extras
  geometry_msgs sensor_msgs std_msgs visualization_msgs
  rclcpp tf tf2_eigen urdf
  mavros mavros_msgs
)

## Declare a cpp executable
add_executable(visualization
  src/visualization.cpp
)
ament_target_dependencies(visualization
  # geometry_msgs sensor_msgs std_msgs visualization_msgs
  # rclcpp tf tf2_eigen urdf
  # mavros mavros_msgs
)

add_executable(servo_state_publisher
  src/servo_state_publisher.cpp
)
ament_target_dependencies(servo_state_publisher
  # geometry_msgs sensor_msgs std_msgs visualization_msgs
  # rclcpp tf tf2_eigen urdf
  # mavros mavros_msgs
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)

## Mark executables and/or libraries for installation
install(TARGETS mavros_extras visualization servo_state_publisher
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mavros_extras.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

##################################
## ament specific configuration ##
##################################
ament_export_dependencies(geometry_msgs mavros mavros_msgs rclcpp sensor_msgs std_msgs visualization_msgs Boost)
ament_package()

# vim: ts=2 sw=2 et:
