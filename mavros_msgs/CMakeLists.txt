cmake_minimum_required(VERSION 3.10)
project(mavros_msgs)

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

set(msg_FILES
  ADSBVehicle.msg
  ActuatorControl.msg
  Altitude.msg
  AttitudeTarget.msg
  BatteryStatus.msg
  CamIMUStamp.msg
  CommandCode.msg
  CompanionProcessStatus.msg
  OnboardComputerStatus.msg
  DebugValue.msg
  ExtendedState.msg
  FileEntry.msg
  GlobalPositionTarget.msg
  HilActuatorControls.msg
  HilControls.msg
  HilGPS.msg
  HilSensor.msg
  HilStateQuaternion.msg
  HomePosition.msg
  LandingTarget.msg
  LogData.msg
  LogEntry.msg
  ManualControl.msg
  Mavlink.msg
  MountControl.msg
  OpticalFlowRad.msg
  OverrideRCIn.msg
  Param.msg
  ParamValue.msg
  PositionTarget.msg
  RCIn.msg
  RCOut.msg
  RTCM.msg
  RadioStatus.msg
  State.msg
  StatusText.msg
  Thrust.msg
  TimesyncStatus.msg
  Trajectory.msg
  VfrHud.msg
  VehicleInfo.msg
  Vibration.msg
  Waypoint.msg
  WaypointList.msg
  WaypointReached.msg
  WheelOdomStamped.msg
)
list(TRANSFORM msg_FILES PREPEND msg/)

set(srv_FILES
  CommandBool.srv
  CommandHome.srv
  CommandInt.srv
  CommandLong.srv
  CommandTOL.srv
  CommandTriggerControl.srv
  CommandTriggerInterval.srv
  FileChecksum.srv
  FileClose.srv
  FileList.srv
  FileMakeDir.srv
  FileOpen.srv
  FileRead.srv
  FileRemove.srv
  FileRemoveDir.srv
  FileRename.srv
  FileTruncate.srv
  FileWrite.srv
  LogRequestData.srv
  LogRequestEnd.srv
  LogRequestList.srv
  MountConfigure.srv
  MessageInterval.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
  SetMavFrame.srv
  SetMode.srv
  StreamRate.srv
  VehicleInfoGet.srv
  WaypointClear.srv
  WaypointPull.srv
  WaypointPush.srv
  WaypointSetCurrent.srv
)
list(TRANSFORM srv_FILES PREPEND srv/)

# add_action_files(
#   DIRECTORY action
#   FILES
#   Action1.action
#   Action2.action
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_FILES} ${srv_FILES}
  DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs builtin_interfaces)

ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")
