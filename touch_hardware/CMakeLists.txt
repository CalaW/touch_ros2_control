cmake_minimum_required(VERSION 3.16)
project(touch_hardware LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-attributes)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenHaptics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

generate_parameter_library(
  touch_driver_parameters
  src/touch_driver_parameters.yaml
)

add_library(touch_hardware_driver SHARED
  src/device.cpp
)
target_compile_features(touch_hardware_driver PUBLIC cxx_std_17)
target_include_directories(touch_hardware_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(touch_hardware_driver PRIVATE
  OpenHaptics::HD
  OpenHaptics::HL
)

add_library(touch_driver_component SHARED
  src/touch_driver_node.cpp
)
target_compile_features(touch_driver_component PUBLIC cxx_std_17)
target_include_directories(touch_driver_component PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(touch_driver_component
  Eigen3
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
)
target_link_libraries(touch_driver_component
  touch_hardware_driver
  touch_driver_parameters
)
rclcpp_components_register_node(
  touch_driver_component
  PLUGIN "touch_hardware::DeviceDriver"
  EXECUTABLE touch_driver
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(TARGETS touch_hardware_driver touch_driver_component touch_driver_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS touch_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_dependencies(generate_parameter_library)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2_ros)
ament_package()
