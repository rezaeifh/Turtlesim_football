cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_football)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlesim_football
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(prepare_environment_with_goalkeeper src/prepare_environment_with_goalkeeper.cpp)
add_dependencies(prepare_environment_with_goalkeeper ${prepare_environment_with_goalkeeper_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(prepare_environment_with_goalkeeper
   ${catkin_LIBRARIES})
   
add_executable(prepare_environment_without_goalkeeper src/prepare_environment_without_goalkeeper.cpp)
add_dependencies(prepare_environment_without_goalkeeper ${prepare_environment_without_goalkeeper_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(prepare_environment_without_goalkeeper
   ${catkin_LIBRARIES})
   
   
add_executable(turtlesim_football_run_with_goalkeeper src/turtlesim_football_run_with_goalkeeper.cpp)
add_dependencies(turtlesim_football_run_with_goalkeeper ${turtlesim_football_run_with_goalkeeper_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlesim_football_run_with_goalkeeper
   ${catkin_LIBRARIES})
   
add_executable(turtlesim_football_run_without_goalkeeper src/turtlesim_football_run_without_goalkeeper.cpp)
add_dependencies(turtlesim_football_run_without_goalkeeper ${turtlesim_football_run_without_goalkeeper_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlesim_football_run_without_goalkeeper
   ${catkin_LIBRARIES})
