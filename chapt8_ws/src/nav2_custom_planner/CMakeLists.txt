cmake_minimum_required(VERSION 3.8)
project(nav2_custom_planner)

# [修改02] 生成实际编译命令，供 VS Code/clangd 正确解析头文件路径。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
# [修改02] 显式查找本包直接使用的依赖，避免依赖 nav2_core 的间接传递。
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 包含头文件目录
include_directories(include)
# 定义库名称
set(library_name ${PROJECT_NAME}_plugin)
# 创建共享库
add_library(${library_name} SHARED  src/nav2_custom_planner.cpp)
# [修改02] 明确使用 Jazzy 常用的 C++17 标准，并将依赖作用于这个共享库目标。
target_compile_features(${library_name} PUBLIC cxx_std_17)
ament_target_dependencies(${library_name}
  nav2_core
  pluginlib
  nav2_util
  nav2_costmap_2d
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  geometry_msgs
  nav_msgs
)
# 安装库文件到指定目录
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# 安装头文件到指定目录
install(DIRECTORY include/
  DESTINATION include/ )
# 导出插件描述文件
pluginlib_export_plugin_description_file(nav2_core custom_planner_plugin.xml)

# [修改02] 将修改说明一同安装，便于安装后查阅配置和调试方法。
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
