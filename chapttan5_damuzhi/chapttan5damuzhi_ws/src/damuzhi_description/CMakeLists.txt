cmake_minimum_required(VERSION 3.8)
project(damuzhi_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 把 URDF 和 STL 网格安装到 share/damuzhi_description 下，
# 这样 package://damuzhi_description/... 才能在运行时被解析。
# （包名要和食指工程的 moveit_description 区分开，否则两个工作空间同时 source 时会互相顶掉）
install(
  DIRECTORY urdf meshes
  DESTINATION share/damuzhi_description
)

ament_package()
