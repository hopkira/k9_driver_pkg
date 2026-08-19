cmake_minimum_required(VERSION 3.16)
project(k9_drive_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(k9_roboclaw_hardware SHARED
  src/k9_roboclaw_hardware.cpp
  src/roboclaw_transport.cpp
)
target_include_directories(k9_roboclaw_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(k9_roboclaw_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
  std_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface k9_roboclaw_hardware.xml)

install(TARGETS k9_roboclaw_hardware
  EXPORT export_k9_roboclaw_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config urdf launch docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE NOTICE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_drive_math test/test_drive_math.cpp)
  if(TARGET test_drive_math)
    target_include_directories(test_drive_math PRIVATE include)
  endif()
endif()

ament_export_targets(export_k9_roboclaw_hardware HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
