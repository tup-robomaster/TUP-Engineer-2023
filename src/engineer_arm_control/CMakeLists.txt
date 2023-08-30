# cmake_minimum_required(VERSION 3.8)
# project(engineer_middle_processor)

cmake_minimum_required(VERSION 3.5)
project(engineer_urdf)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY 
  # meshes
  urdf
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
