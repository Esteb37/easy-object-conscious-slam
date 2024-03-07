cmake_minimum_required(VERSION 3.5)
project(ocslam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

ament_python_install_package(${PROJECT_NAME})

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver ocslam.xml)

# Add each node to the add_executable() command
# add_executable(Socket src/Socket.cpp)
#add_executable(Capture src/Capture.cpp)
add_executable(main src/main.cpp)


# Link each node to the necessary libraries
# target_link_libraries(Socket ${PROJECT_NAME})
#target_link_libraries(Capture ${PROJECT_NAME})
target_link_libraries(main ${PROJECT_NAME})


add_library(
  ${PROJECT_NAME}
  SHARED
  src/Robot.cpp
  src/main.cpp
  # src/Socket.cpp
  # src/Capture.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
include_directories(${cv_bridge_INCLUDE_DIRS})
target_link_libraries(main ${cv_bridge_LIBRARIES})
# target_link_libraries(Socket ${Boost_LIBRARIES})



ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  nav_msgs
  cv_bridge
  rosidl_typesupport_cpp
)
install(TARGETS
  ${PROJECT_NAME}
  # Socket
  # Capture
  main
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
  src
  rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(
  include
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(PROGRAMS
  scripts/YOLO.py
  scripts/Utils.py
  scripts/Projection.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()