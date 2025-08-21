cmake_minimum_required(VERSION 3.8)
project(carlikebot_bringup)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(generic_steering_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Build the Plugin Library ---
add_library(
  carlike_kinematics SHARED
  src/car_like_kinematics.cpp
)

target_include_directories(carlike_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(carlike_kinematics PUBLIC
  generic_steering_controller::generic_steering_controller
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

# --- CRITICAL: Export the Plugin ---
pluginlib_export_plugin_description_file(generic_steering_controller plugins.xml)

# --- Installation ---
install(
  TARGETS carlike_kinematics
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()