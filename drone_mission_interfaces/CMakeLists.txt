cmake_minimum_required(VERSION 3.8)
project(drone_mission_interfaces)

find_package(ament_cmake REQUIRED)                # ROS build tool that works on top of CMake
find_package(rosidl_default_generators REQUIRED)  # ROS tool to generate code from simple file
find_package(action_msgs REQUIRED)                # Pre-made message types

# Generate code from MissionCommand
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MissionCommand.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
