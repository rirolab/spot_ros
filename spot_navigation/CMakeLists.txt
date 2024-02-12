cmake_minimum_required(VERSION 2.8.3)
project(spot_navigation)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros rospy tf)

catkin_package()

install(
  DIRECTORY config launch rviz src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_filtering
  src/lidar_filtering.cpp
)
target_link_libraries(lidar_filtering
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(rbq_controller
  src/rbq_controller.cpp
)
target_link_libraries(rbq_controller
  ${catkin_LIBRARIES}
)

add_executable(robot_to_block
  src/robot_to_block.cpp
)
target_link_libraries(robot_to_block
  ${catkin_LIBRARIES}
)

add_executable(add_point 
  src/add_point.cpp
)

target_link_libraries(add_point
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf_LIBRARIES}
)
