cmake_minimum_required(VERSION 3.5)
project(piper_moveit_config)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fake_hardware_interface.py
  scripts/custom_js_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
