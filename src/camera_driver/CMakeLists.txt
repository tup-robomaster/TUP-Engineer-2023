cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(global_interface REQUIRED)
find_package(global_user REQUIRED)

set(dependencies 
  rclcpp
  rclcpp_components
  std_msgs
  camera_info_manager
  image_transport
  camera_calibration_parsers
  OpenCV
  Eigen3
  yaml-cpp

  global_user
  global_interface 
)

include_directories(include)
include_directories(include ${cv_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/usb_cam_pub.cpp
  src/usb_cam.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "camera_driver::UsbCamNode")

add_executable(usb_cam_pub
  src/usb_cam_pub.cpp
  src/usb_cam.cpp
)

target_link_libraries(usb_cam_pub 
  ${PROJECT_NAME}
  rclcpp::rclcpp 
  ${std_msgs_TARGETS}
)

ament_target_dependencies(usb_cam_pub
  ${dependencies}
)

install(TARGETS 
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  usb_cam_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
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
