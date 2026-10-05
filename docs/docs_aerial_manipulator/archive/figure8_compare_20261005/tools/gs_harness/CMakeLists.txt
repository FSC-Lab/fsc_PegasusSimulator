# gs_fig8_harness: the real arm-GS EE Trajectory panel, offscreen (2026-10-05).
# Links the station's own static library from the workspace build, so build the
# station first (colcon build --packages-select utils_custom_ground_station).
#
#   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
#   cmake -S . -B <build dir> && cmake --build <build dir> -j
cmake_minimum_required(VERSION 3.16)
project(gs_fig8_harness)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(DEFINED ENV{QT_HOME})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{QT_HOME}")
elseif(EXISTS "$ENV{HOME}/Qt/6.8.3/gcc_64")
  list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/Qt/6.8.3/gcc_64")
endif()
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_RPATH_USE_LINK_PATH TRUE)

set(GS_WS "$ENV{HOME}/ros2_ws" CACHE PATH "workspace holding fsc_open_manipulator")
set(GS_SRC "${GS_WS}/src/fsc_open_manipulator/utils_custom_ground_station/src")
set(GS_LIB "${GS_WS}/build/utils_custom_ground_station/libjoint_plot_core.a")

find_package(ament_cmake REQUIRED)
foreach(p rclcpp rclcpp_action sensor_msgs std_msgs std_srvs geometry_msgs
    control_msgs controller_manager_msgs fsc_autopilot_ros2_msgs)
  find_package(${p} REQUIRED)
endforeach()
find_package(Qt6 REQUIRED COMPONENTS Widgets Charts)

add_executable(gs_fig8_harness gs_fig8_harness.cpp)
target_include_directories(gs_fig8_harness PRIVATE "${GS_SRC}")
# the station is built with the mocap frame; the panel's class layout depends on it
target_compile_definitions(gs_fig8_harness PRIVATE ARM_GUI_HAVE_MOCAP)
target_link_libraries(gs_fig8_harness "${GS_LIB}" Qt6::Widgets Qt6::Charts)
ament_target_dependencies(gs_fig8_harness
  rclcpp rclcpp_action sensor_msgs std_msgs std_srvs geometry_msgs
  control_msgs controller_manager_msgs fsc_autopilot_ros2_msgs)
