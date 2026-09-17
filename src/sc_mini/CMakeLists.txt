cmake_minimum_required(VERSION 3.8)
project(sc_mini)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(sc_mini
  src/sc_mini.cpp
  src/serial_port.cpp
)
ament_target_dependencies(sc_mini
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS sc_mini
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES sc_mini.rules
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
