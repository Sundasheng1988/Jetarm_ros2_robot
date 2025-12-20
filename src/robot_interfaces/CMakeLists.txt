cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# 可选：设置 C / C++ 标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ===== 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ===== 定义 msg 文件 =====
set(msg_files
  "msg/EnvObject.msg"
  "msg/EnvObjectArray.msg"
)

# ===== 生成接口 =====
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# 运行时依赖导出
ament_export_dependencies(rosidl_default_runtime)

ament_package()

