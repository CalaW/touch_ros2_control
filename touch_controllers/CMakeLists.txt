cmake_minimum_required(VERSION 3.16)
project(touch_controllers LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(touch_hardware REQUIRED)

add_library(touch_controllers_plugins SHARED
  src/null_controller.cpp
  src/direct_force_controller.cpp
  src/impedance_controller.cpp
)
target_compile_features(touch_controllers_plugins PUBLIC cxx_std_17)
target_include_directories(touch_controllers_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(touch_controllers_plugins
  Eigen3
  pluginlib
  touch_hardware
)
target_link_libraries(touch_controllers_plugins
  touch_hardware::touch_hardware_base
)

pluginlib_export_plugin_description_file(touch_hardware touch_controllers_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS touch_controllers_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  ament_add_gtest(test_controllers test/test_controllers.cpp)
  if(TARGET test_controllers)
    target_include_directories(test_controllers PRIVATE include)
    ament_target_dependencies(test_controllers Eigen3 touch_hardware)
    target_link_libraries(test_controllers
      touch_controllers_plugins
      touch_hardware::touch_hardware_base
    )
  endif()
endif()

ament_package()
