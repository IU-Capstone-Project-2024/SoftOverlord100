cmake_minimum_required(VERSION 3.8)
project(overlord100_map_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(overlord100_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(nav2_map_server REQUIRED)

# include_directories(include ${rclcpp_INCLUDE_DIRS})


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



# cmake_minimum_required(VERSION 3.5)
# project(overlord100_map_saver)

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(nav2_map_server REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)

# # Include directories
# include_directories(include ${rclcpp_INCLUDE_DIRS})

# # Add executable
# add_executable(map_saver_node src/map_saver.cpp)

# # Link libraries
# ament_target_dependencies(map_saver_node rclcpp nav2_map_server std_msgs nav_msgs)

# # Install
# install(TARGETS
#   map_saver_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# # install(DIRECTORY launch params
# #   DESTINATION share/${PROJECT_NAME}
# # )

# ament_package()
