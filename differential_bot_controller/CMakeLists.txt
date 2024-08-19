cmake_minimum_required(VERSION 3.8)
project(differential_bot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(differential_bot_controller_parameters
  src/differential_bot_controller_parameter.yaml
)

# adding our main cpp code in library dir after building the pkg
add_library(differential_bot_controller SHARED
  src/differential_bot_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(differential_bot_controller PUBLIC cxx_std_17)

#adding target dependencies (the library of our main code)
target_include_directories(differential_bot_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/differential_bot_controller>
)
target_link_libraries(differential_bot_controller PUBLIC differential_bot_controller_parameters)

#installing depedencies
ament_target_dependencies(differential_bot_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(differential_bot_controller PRIVATE "DIFFERENTIAL_BOT_CONTROLLER_BUILDING_DLL")

# important for hardware interfacing 
pluginlib_export_plugin_description_file(controller_interface differential_bot_plugin.xml)

 
install(
  DIRECTORY include/
  DESTINATION include/differential_bot_controller
)
install(TARGETS differential_bot_controller differential_bot_controller_parameters
  EXPORT export_differential_bot_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# only required if we are testing code (need to add test dir as an extra)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
