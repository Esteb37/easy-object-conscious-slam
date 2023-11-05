cmake_minimum_required(VERSION 3.5)
project(ocslam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver ocslam.xml)

# Add each node to the add_executable() command
add_executable(Follower src/Follower.cpp)

# Link each node to the necessary libraries
target_link_libraries(Follower ${PROJECT_NAME})


add_library(
  ${PROJECT_NAME}
  SHARED
  src/Robot.cpp
  src/Follower.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  Follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(
  include
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()