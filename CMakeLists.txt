cmake_minimum_required(VERSION 2.8.3)
project(ranged_finder)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RangedFinder.msg
)
add_service_files(
  FILES
  getSensorData.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
 INCLUDE_DIRS inc
 LIBRARIES lib
 CATKIN_DEPENDS roscpp
)

include_directories(inc/ ${catkin_INCLUDE_DIRS} )

add_executable(
measure3sensors_node 
src/measure3sensors.cpp 
src/jetsonGPIO.c 
)

add_executable(
measure2sensors_node 
src/measure2ensors.cpp 
src/jetsonGPIO.c 
)

add_executable(
measure1sensor_node 
src/measure1sensor.cpp 
src/jetsonGPIO.c 
)

FIND_LIBRARY(rangedFinder_lib VL53L0X_Rasp lib)

target_link_libraries(
  measure3sensors_node
  ${rangedFinder_lib}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  measure2sensors_node
  ${rangedFinder_lib}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  measure1sensor_node 
  ${rangedFinder_lib}
  ${catkin_LIBRARIES}
)






