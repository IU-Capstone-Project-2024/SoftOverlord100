cmake_minimum_required(VERSION 3.8)
project(overlord100_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelsData.msg"
  "msg/LogMessage.msg"
  "msg/Mode.msg"
  "msg/MapName.msg"
  "msg/SaveMap.msg"
  "srv/ChangeMode.srv"
  "srv/PubliishMap.srv"
  DEPENDENCIES  # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_package()
