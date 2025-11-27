cmake_minimum_required(VERSION 3.8)
project(vacuscan_bringup)

find_package(ament_cmake REQUIRED)

# We only install launch + rviz files (no code)
install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
