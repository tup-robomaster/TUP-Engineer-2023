cmake_minimum_required(VERSION 3.8)
project(stone_station_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
# custom package
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
)

set(dependencies 
  rclcpp
  rclpy
  OpenCV 
  message_filters 
  rclcpp_components
  image_transport
  sensor_msgs
  OpenVINO
  Ceres
  Eigen3
  cv_bridge
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros 
  yaml-cpp

  global_interface
  global_user
)

add_library(${PROJECT_NAME} SHARED
  src/detector_node.cpp
  src/detector.cpp
  src/inference.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}  
)

target_link_libraries(${PROJECT_NAME}
  openvino::runtime 
  yaml-cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

add_executable(${PROJECT_NAME}_node
  src/detector_node.cpp
  src/detector.cpp
  src/inference.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  openvino::runtime 
  ${Eigen3_LIBRARIES}
  yaml-cpp
  fmt
)

rclcpp_components_register_nodes(${PROJECT_NAME} 
  PLUGIN "stone_station_detector::DetectorNode" 
  # EXECUTABLE stone_station_detector_node
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
