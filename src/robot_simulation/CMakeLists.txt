cmake_minimum_required(VERSION 3.8)
project(robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts to lib/<pkg> so ros2 run can find them
install(PROGRAMS
  scripts/twist_stamper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
