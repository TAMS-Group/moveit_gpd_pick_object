cmake_minimum_required(VERSION 2.8.3)
project(moveit_gpd_pick_object)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_core
  roscpp
  std_msgs
  sensor_msgs
  gpd
  tf
  pcl_ros
  message_filters
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_object_demo src/pick_object_demo.cpp)
target_link_libraries(pick_object_demo ${catkin_LIBRARIES})

add_executable(plan_grasps_service src/plan_grasps_service.cpp)
target_link_libraries(plan_grasps_service ${catkin_LIBRARIES})

