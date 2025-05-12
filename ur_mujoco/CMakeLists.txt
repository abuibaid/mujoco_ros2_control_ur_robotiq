cmake_minimum_required(VERSION 3.10.2)
project(ur_mujoco)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY ur5e DESTINATION share/${PROJECT_NAME})

ament_package()
