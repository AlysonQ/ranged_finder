cmake_minimum_required(VERSION 2.8.3)
project(ranged_finder)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

catkin_package(
 INCLUDE_DIRS inc
 LIBRARIES lib
 CATKIN_DEPENDS roscpp
)

include_directories(inc/ ${catkin_INCLUDE_DIRS} )

add_message_files(
  FILES
  RangedFinder.msg
)


add_executable(ranged_finder_node 
src/measure.cpp 
src/jetsonGPIO.c 
)

FIND_LIBRARY(rangedFinder_lib VL53L0X_Rasp lib)
target_link_libraries(ranged_finder_node
  ${rangedFinder_lib}
  ${catkin_LIBRARIES}
)
