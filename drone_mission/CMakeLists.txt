cmake_minimum_required(VERSION 3.8)
project(drone_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(drone_mission_interfaces REQUIRED)

# Compile path.cpp into library called mission_path and tell the compiler where its headers live
add_library(mission_path src/path.cpp)
target_include_directories(mission_path PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile mission_server into binary and link all it needs to compile
add_executable(mission_server src/mission_server.cpp)
target_link_libraries(mission_server
  mission_path
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${px4_msgs_TARGETS}
  ${drone_mission_interfaces_TARGETS})

# Compile mission_cli into binary and link all it needs to compile
add_executable(mission_cli src/mission_cli.cpp)
target_link_libraries(mission_cli
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${drone_mission_interfaces_TARGETS})

install(TARGETS mission_server mission_cli DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
