cmake_minimum_required(VERSION 3.8)
project(fitrobot)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  launch
  common
  script
  data
  maps
  masks
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()