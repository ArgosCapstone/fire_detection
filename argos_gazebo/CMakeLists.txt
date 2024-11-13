cmake_minimum_required(VERSION 3.5)
project(argos_gazebo)

# ...existing code...

# Ensure ament_python is used
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(argos_gazebo)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/argos_gazebo/launch
)

# ...existing code...

ament_package()