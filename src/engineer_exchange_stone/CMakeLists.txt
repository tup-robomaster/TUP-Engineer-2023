cmake_minimum_required(VERSION 3.8)
project(stone_station_detector)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
  
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++17 -Os -pthread -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set(CMAKE_PREFIX_PATH /opt/intel/openvino_2021/deployment_tools/inference_engine/share)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)

find_package(cv_bridge REQUIRED)

# custom package
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

set(dependencies 
  rclcpp
  rclpy
  OpenCV 
  message_filters 
  image_transport
  sensor_msgs
  InferenceEngine 
  OpenVINO
  Ceres
  Eigen3
  cv_bridge
  geometry_msgs
  global_interface
  global_user
  
  yaml-cpp 
  fmt
)

add_library(${PROJECT_NAME} SHARED
  src/detector_node.cpp
  src/detector.cpp
  src/inference.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}  
)
  
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

set(node_plugins "")
rclcpp_components_register_nodes(${PROJECT_NAME}
  PLUGIN "stone_station_detector::detector_node"
  EXECUTABLE stone_station_detector
)
set(node_plugins "${node_plugins}stone_station_detector::detector_node;&<TARGET_FILE:detector_node\n")

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
  ${InferenceEngine_LIBRARIES}
  openvino::runtime 
  ${Eigen3_LIBRARIES}
  yaml-cpp
  glog.so 
  fmt.so
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib # 静态库安装路径
  LIBRARY DESTINATION lib # 动态库安装路径
  RUNTIME DESTINATION bin # 可执行文件安装路径
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
