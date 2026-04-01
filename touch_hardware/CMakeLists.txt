cmake_minimum_required(VERSION 3.16)
project(touch_hardware LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the touch_hardware repository?")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenHaptics REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(${PROJECT_NAME} SHARED
  src/touch_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  OpenHaptics::HD
  OpenHaptics::HL
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface touch_hardware_plugin.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_touch_system test/test_touch_system.cpp)
  target_compile_features(test_touch_system PUBLIC cxx_std_17)
  target_link_libraries(test_touch_system
    ${PROJECT_NAME}
    pluginlib::pluginlib
    hardware_interface::hardware_interface
  )
  ament_target_dependencies(test_touch_system
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
endif()


## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()