cmake_minimum_required(VERSION 2.8.3)
project(move1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY demo driver robot_para rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# add_executable(chemical_move src/chemical_move.cpp)
# add_dependencies(chemical_move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(chemical_move ${catkin_LIBRARIES})
      
#add_executable(set_nav_goal src/set_nav_goal(5-09).cpp)
#add_dependencies(set_nav_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} $#{catkin_EXPORTED_TARGETS})
#target_link_libraries(set_nav_goal ${catkin_LIBRARIES})
# add_executable(NavigateToTarget src/navigation.cpp src/navigation_main.cpp)
# add_dependencies(set_nav_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} $#{catkin_EXPORTED_TARGETS})
# target_link_libraries(NavigateToTarget ${catkin_LIBRARIES})

add_executable(NavigateToTarget src/navigation_0707.cpp src/navigation_main.cpp)
add_dependencies(NavigateToTarget ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(NavigateToTarget ${catkin_LIBRARIES})



add_executable(robot_monitor src/monitor2.cpp)
add_dependencies(robot_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_monitor ${catkin_LIBRARIES})
# add_executable(NavigateToTarget src/navigation_0708.cpp src/navigation_main.cpp)
# add_dependencies(NavigateToTarget ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(NavigateToTarget ${catkin_LIBRARIES})

# add_executable(set_nav_goal_2ban src/set_nav_goal_2ban.cpp)
# add_dependencies(set_nav_goal_2ban ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(set_nav_goal_2ban ${catkin_LIBRARIES})

# add_executable(set_nav_goal_0510-2 src/set_nav_goal_0510-2.cpp)
# add_dependencies(set_nav_goal_0510-2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(set_nav_goal_0510-2 ${catkin_LIBRARIES})

# add_executable(set_nav_goal_0510-1 src/set_nav_goal_0510-1.cpp)
# add_dependencies(set_nav_goal_0510-1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(set_nav_goal_0510-1 ${catkin_LIBRARIES})

