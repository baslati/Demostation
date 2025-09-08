cmake_minimum_required(VERSION 3.10)
project(custom_ur_description)

find_package(ament_cmake REQUIRED)

# EXPLIZIT die Ordner installieren, die du hast:
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
