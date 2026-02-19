cmake_minimum_required(VERSION 3.8)
project(panda_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch panda_controller # assuming urdf files are in the urdf directory
  DESTINATION share/${PROJECT_NAME} # install to share/my_robot_description
)

# if you want to make a node executable
install(
  PROGRAMS
  panda_controller/slider_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
