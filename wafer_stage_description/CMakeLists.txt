cmake_minimum_required(VERSION 3.8)
project(wafer_stage_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf xacro rviz meshes 
    DESTINATION share/${PROJECT_NAME})
ament_package()
