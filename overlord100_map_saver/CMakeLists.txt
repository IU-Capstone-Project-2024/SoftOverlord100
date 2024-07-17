cmake_minimum_required(VERSION 3.8)
project(overlord100_map_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(overlord100_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(map_saver src/map_saver.cpp)

ament_target_dependencies(
  map_saver
  rclcpp
  overlord100_msgs
)

install(TARGETS 
  map_saver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
