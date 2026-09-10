cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

find_package(ament_cmake REQUIRED)                # ROS build tool that works on top of CMake
find_package(rosidl_default_generators REQUIRED)  # ROS tool to generate code from simple file

# Only the PX4 interfaces drone_mission uses, copied from px4_msgs.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OffboardControlMode.msg"
  "msg/TelemetryStatus.msg"
  "msg/TrajectorySetpoint.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleCommandAck.msg"
  "msg/VehicleLandDetected.msg"
  "msg/VehicleLocalPosition.msg"
  "msg/VehicleStatus.msg"
  "srv/VehicleCommand.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
