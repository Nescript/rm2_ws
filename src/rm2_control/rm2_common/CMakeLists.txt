cmake_minimum_required(VERSION 3.8)
project(rm2_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPEND_PACKAGES
        rclcpp
        rclcpp_lifecycle
        controller_manager_msgs
        control_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        rm2_msgs
        Eigen3
        joint_limits
        hardware_interface
        tf2
        tf2_ros
        tf2_msgs
        message_filters
        imu_complementary_filter
        realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${DEPEND_PACKAGES})
  find_package(${Dependency} REQUIRED)
endforeach ()

file(GLOB DECISION "include/rm2_common/decision/*")
file(GLOB FILTERS "include/rm2_common/filters/*")
file(GLOB HARDWARE_HANDLE "include/rm2_common/hardware_handle/*")
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/filter/*.cpp")
add_library(${PROJECT_NAME} SHARED
       ${DECISION}
       ${FILTERS}
       ${HARDWARE_HANDLE}
       ${SOURCES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPEND_PACKAGES})

# INSTALL
install(
        DIRECTORY include/
        DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        ${DEPEND_PACKAGES}
)
ament_package()
