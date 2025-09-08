cmake_minimum_required(VERSION 3.10)
project(custom_ur_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(dir IN ITEMS config launch rviz srdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
  endif()
endforeach()

ament_package()

