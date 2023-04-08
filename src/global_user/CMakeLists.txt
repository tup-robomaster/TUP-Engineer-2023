cmake_minimum_required(VERSION 3.8)
project(global_user)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp_components
  geometry_msgs
  sensor_msgs
  rclcpp
  std_msgs
  visualization_msgs
  OpenCV
  Eigen3
  yaml-cpp

  tf2_geometry_msgs
  tf2_ros 
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 添加源文件，生成库
add_library(${PROJECT_NAME} SHARED
  src/global_user.cpp
  src/coordsolver.cpp
)

# 用于代替传统的target_link_libraries
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  yaml-cpp
)
   
target_link_libraries(${PROJECT_NAME}
  yaml-cpp
)
# 注册 导出头文件
install(
  DIRECTORY include
  DESTINATION include
)

# 注册 导出库文件
install(
  TARGETS ${PROJECT_NAME} # 告诉ros2有这么个目标（可执行文件或者库）
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  config 
  DESTINATION share/${PROJECT_NAME}
)    
        
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
      
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
