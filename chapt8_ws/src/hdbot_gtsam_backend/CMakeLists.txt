cmake_minimum_required(VERSION 3.18)
project(hdbot_gtsam_backend)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Keep non-system GTSAM library paths usable after colcon's install step too.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
find_package(ament_cmake REQUIRED)
find_package(GTSAM CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(slam_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hdbot_gnss_msgs REQUIRED)
# Jazzy's modern slam_toolbox exports omit these implementation libraries.
find_library(KARTO_LIBRARY kartoSlamToolbox HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
find_library(ASYNC_SLAM_LIBRARY async_slam_toolbox HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)
find_library(TOOLBOX_COMMON_LIBRARY toolbox_common HINTS "${slam_toolbox_DIR}/../../../lib" REQUIRED)

# Check the actual class, nav-frame overload and exported symbol, not just a version.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES gtsam)
check_cxx_source_compiles("
#include <gtsam/navigation/DopplerFactor.h>
int main() {
 gtsam::Point3 p(2e7,0,0), r(6e6,0,0), z(0,0,0);
 gtsam::DopplerFactorArm f(0,1,2,3,0.0,0.19,p,z,r,z,
   gtsam::Pose3(),z,1.0,0.0,gtsam::noiseModel::Unit::Create(1));
 return f.evaluateError(gtsam::Pose3(),z,0.0,0.0).size()!=1;
}" HAS_OFFICIAL_DOPPLER_ARM)
unset(CMAKE_REQUIRED_LIBRARIES)
if(NOT HAS_OFFICIAL_DOPPLER_ARM)
  message(FATAL_ERROR "GTSAM must contain navigation/DopplerFactor.h and the local-nav DopplerFactorArm overload. See README.md.")
endif()

add_library(hdbot_gtsam_core SHARED src/graph_backend.cpp)
target_include_directories(hdbot_gtsam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hdbot_gtsam_core PUBLIC gtsam)
target_compile_options(hdbot_gtsam_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(hdbot_gtsam_solver SHARED src/gtsam_solver.cpp)
target_link_libraries(hdbot_gtsam_solver hdbot_gtsam_core Boost::serialization
  Boost::thread ${KARTO_LIBRARY})
ament_target_dependencies(hdbot_gtsam_solver slam_toolbox pluginlib rclcpp
  rclcpp_lifecycle sensor_msgs nav_msgs hdbot_gnss_msgs)
pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

add_library(hdbot_gnss_mapper SHARED src/slam_gnss_node.cpp)
target_link_libraries(hdbot_gnss_mapper hdbot_gtsam_solver ${ASYNC_SLAM_LIBRARY} ${TOOLBOX_COMMON_LIBRARY})
ament_target_dependencies(hdbot_gnss_mapper slam_toolbox rclcpp rclcpp_lifecycle geometry_msgs)
add_executable(slam_gnss_node src/main.cpp)
target_link_libraries(slam_gnss_node hdbot_gnss_mapper)
ament_target_dependencies(slam_gnss_node rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend hdbot_gtsam_core)
  ament_add_gtest(test_solver test/test_solver.cpp
    ENV ROS_LOG_DIR=/tmp/hdbot_gnss_test_logs)
  target_link_libraries(test_solver hdbot_gtsam_solver)
  ament_target_dependencies(test_solver slam_toolbox pluginlib rclcpp hdbot_gnss_msgs)
  ament_add_gtest(test_mapper test/test_mapper.cpp
    ENV ROS_LOG_DIR=/tmp/hdbot_gnss_test_logs TIMEOUT 30)
  target_link_libraries(test_mapper hdbot_gnss_mapper)
  ament_target_dependencies(test_mapper rclcpp rclcpp_lifecycle)
endif()
install(TARGETS hdbot_gtsam_core hdbot_gtsam_solver hdbot_gnss_mapper slam_gnss_node
  EXPORT export_backend ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_backend HAS_LIBRARY_TARGET)
ament_export_dependencies(GTSAM slam_toolbox hdbot_gnss_msgs)
ament_package(CONFIG_EXTRAS "cmake/dependencies.cmake")
