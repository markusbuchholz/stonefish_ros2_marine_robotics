cmake_minimum_required(VERSION 3.5)
project(cola2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalDescriptor.msg"
  "msg/Bool6Axis.msg"
  "msg/BodyForceReq.msg"
  "msg/BodyVelocityReq.msg"
  "msg/CaptainStateFeedback.msg"
  "msg/CaptainStatus.msg"
  "msg/DecimalLatLon.msg"
  "msg/DVL.msg"
  "msg/DVLBeam.msg"
  "msg/Float32Stamped.msg"
  "msg/KeyValue.msg"
  "msg/Landmark.msg"
  "msg/Map.msg"
  "msg/MissionState.msg"
  "msg/NavSts.msg"
  "msg/NED.msg"
  "msg/RecoveryAction.msg"
  "msg/RPY.msg"
  "msg/SafetySupervisorStatus.msg"
  "msg/Setpoints.msg"
  "msg/WorldWaypointReq.msg"
  "srv/Action.srv"
  "srv/DigitalOutput.srv"
  "srv/Goto.srv"
  "srv/MaxJoyVelocity.srv"
  "srv/Mission.srv"
  "srv/Recovery.srv"
  "srv/Section.srv"
  "srv/String.srv"
  "action/Pilot.action"
  DEPENDENCIES std_msgs sensor_msgs action_msgs geometry_msgs nav_msgs
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()

