cmake_minimum_required(VERSION 2.8.3)
project(amazebot-pathfinding-stage)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_basic_stage src/wall_follower.cpp)
target_link_libraries(maze_basic_stage ${catkin_LIBRARIES})

#add_executable(maze_pro_stage src/astar_solver.cpp) # or .py?
#target_link_libraries(maze_pro_stage ${catkin_LIBRARIES})

add_executable(map_saver_stage src/map_saver.cpp)
target_link_libraries(map_saver_stage ${catkin_LIBRARIES})
