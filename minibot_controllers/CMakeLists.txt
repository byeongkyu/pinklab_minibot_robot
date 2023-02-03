cmake_minimum_required(VERSION 3.8)
project(minibot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(minibot_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  minibot_interfaces
  controller_interface
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  std_srvs
  sensor_msgs
)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/gpio_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
