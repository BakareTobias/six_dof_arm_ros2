cmake_minimum_required(VERSION 3.8)
project(pap_arm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch config worlds # assuming urdf files are in the urdf directory
  DESTINATION share/${PROJECT_NAME} # install to share/my_robot_description
)


ament_package()
