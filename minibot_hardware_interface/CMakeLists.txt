cmake_minimum_required(VERSION 3.8)
project(minibot_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)


## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/minibot_system.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${SERIAL_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${SERIAL_LDFLAGS}
  ${CMAKE_THREAD_LIBS_INIT}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface minibot_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# add_executable(ExampleProject example_project.cpp)
# target_include_directories(ExampleProject PRIVATE ${SERIAL_INCLUDE_DIRS})
# target_link_libraries(ExampleProject PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
