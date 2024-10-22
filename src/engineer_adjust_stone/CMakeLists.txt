cmake_minimum_required(VERSION 3.8)
project(stone_control)

set(CMAKE_BUILD_TYPE=Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(cv_bridge REQUIRED)
# custom package
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclpy
  OpenCV 
  message_filters 
  image_transport
  sensor_msgs
  Ceres
  Eigen3
  cv_bridge
  global_interface
  global_user
  yaml-cpp 
  fmt
)

add_library(${PROJECT_NAME} SHARED
  src/stone_control.cpp
  src/stone_control_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Eigen3_LIBRARIES}
  yaml-cpp 
  glog 
  fmt
)
  
add_executable(${PROJECT_NAME}_node src/stone_control_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "stone_control::stone_control_node"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  # config
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
