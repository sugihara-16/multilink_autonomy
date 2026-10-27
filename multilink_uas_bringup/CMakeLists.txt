cmake_minimum_required(VERSION 3.8)
project(multilink_uas_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY native DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "logs" EXCLUDE
  PATTERN "handoff_reference" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "state.json" EXCLUDE
  PATTERN ".supervisor.lock" EXCLUDE
  PATTERN ".articulated.lock" EXCLUDE
  PATTERN "*.before-workspace-migration" EXCLUDE)
ament_package()
