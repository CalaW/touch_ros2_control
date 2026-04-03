cmake_minimum_required(VERSION 3.8)
project(touch_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf ros2_control meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_touch_description_xacro test/test_urdf_xacro.py)
endif()

ament_package()
