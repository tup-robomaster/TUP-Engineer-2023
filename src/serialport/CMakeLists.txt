cmake_minimum_required(VERSION 3.8)
project(serialport)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenCV REQUIRED)
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
  
set(node_plugins "")
  
add_library(${PROJECT_NAME} SHARED
      src/CRC_Check.cpp
      src/${PROJECT_NAME}.cpp
      src/data_transform.cpp
      src/${PROJECT_NAME}_node.cpp
)

set(dependencies 
  rclcpp
  rclcpp_components
  std_msgs
  OpenCV
  global_interface
  global_user
  rclcpp_lifecycle 
  visualization_msgs 
  angles 
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)

include_directories(include)
add_executable(${PROJECT_NAME}_node
  src/CRC_Check.cpp
  src/${PROJECT_NAME}.cpp
  src/data_transform.cpp
  src/${PROJECT_NAME}_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node 
  yaml-cpp
)

# register component nodes
rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "serialport::serial_driver"
  EXECUTABLE ${PROJECT_NAME}_node  
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
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
