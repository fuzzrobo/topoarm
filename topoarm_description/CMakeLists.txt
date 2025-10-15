cmake_minimum_required(VERSION 3.5)
project(topoarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz gazebo ros2_control urdf meshes 
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
