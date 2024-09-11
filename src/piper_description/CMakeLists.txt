cmake_minimum_required(VERSION 3.5)
project(piper_description)

# 找到ament_cmake来构建这个包
find_package(ament_cmake REQUIRED)

# 安装文件夹
install(DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

# 包的描述
ament_package()
