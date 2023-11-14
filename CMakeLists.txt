cmake_minimum_required(VERSION 3.5)
project(anf_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch bb8 intro DESTINATION share/${PROJECT_NAME})

file(GLOB scripts src/*)
install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME})

ament_package()
