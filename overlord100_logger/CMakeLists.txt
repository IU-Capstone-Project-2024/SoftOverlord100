cmake_minimum_required(VERSION 3.8)
project(overlord100_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(overlord100_msgs REQUIRED)


add_executable(log_collector src/log_collector.cpp)
ament_target_dependencies(
  log_collector
  rclcpp
  overlord100_msgs
)

install(TARGETS 
  log_collector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
