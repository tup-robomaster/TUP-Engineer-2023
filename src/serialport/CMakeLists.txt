cmake_minimum_required(VERSION 3.8)
project(serialport)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# SET(CMAKE_BUILD_TYPE "Debug")  
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# ## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# ## enforcing cleaner code.
# add_definitions(-Wall -Werror)

# ## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(OpenCV REQUIRED)
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
  
set(node_plugins "")
  

set(dependencies 
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  OpenCV
  global_interface
  global_user
  tf2_geometry_msgs
  tf2_ros

  rclcpp_lifecycle 
  laser_geometry 
  message_filters 
  sensor_msgs 
  tf2 
  tf2_sensor_msgs 
  visualization_msgs 
  angles 
  robot_state_publisher 
)

include_directories(include)

add_executable(serial_driver
  include/${PROJECT_NAME}/crc_check.hpp
  include/${PROJECT_NAME}/crc_check_old.hpp

  include/${PROJECT_NAME}/packet.hpp
  # include/${PROJECT_NAME}/packet.h
  include/${PROJECT_NAME}/${PROJECT_NAME}_node.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  include/${PROJECT_NAME}/${PROJECT_NAME}_old.hpp

  src/${PROJECT_NAME}/crc_check.cpp
  src/${PROJECT_NAME}/crc_check_old.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}_old.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
  src/${PROJECT_NAME}/${PROJECT_NAME}_node_main.cpp
  # src/${PROJECT_NAME}/crc_check.cpp
  # src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

ament_target_dependencies(serial_driver 
  ${dependencies}
)


install(TARGETS
  serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

# install inlude directories
install(
    DIRECTORY include/
    DESTINATION include
)

# install launch resource config directory
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)


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


