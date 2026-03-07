cmake_minimum_required(VERSION 3.8)
project(rm2_chassis_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rm2_common REQUIRED)
find_package(rm2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
    rclcpp
    rclcpp_lifecycle
    controller_interface
    hardware_interface
    realtime_tools
    rm2_common
    rm2_msgs
    geometry_msgs
    nav_msgs
    tf2_geometry_msgs
    control_toolbox
    pluginlib
    Eigen3
)

add_library(${PROJECT_NAME} SHARED
    src/chassis_base.cpp
    src/omni.cpp
    src/swerve.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    ${DEPENDENCIES}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

pluginlib_export_plugin_description_file(controller_interface rm2_chassis_controllers_plugins.xml)

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
