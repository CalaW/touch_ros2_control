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
find_package(geometry_msgs REQUIRED)
find_package(OpenHaptics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_base SHARED
  src/controller_base.cpp
)
target_compile_features(${PROJECT_NAME}_base PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_base PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_base
  rclcpp
)

add_executable(touch_driver
  src/touch_driver_node.cpp
)
target_compile_features(touch_driver PUBLIC cxx_std_17)
target_include_directories(touch_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(touch_driver
  Eigen3
  geometry_msgs
  pluginlib
  rclcpp
  sensor_msgs
)
target_link_libraries(touch_driver
  ${PROJECT_NAME}_base
  OpenHaptics::HD
  OpenHaptics::HL
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(TARGETS ${PROJECT_NAME}_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS touch_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_package()
