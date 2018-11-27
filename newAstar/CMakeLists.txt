cmake_minimum_required(VERSION 2.8.3)
project(newAStar)


add_compile_options(-std=c++11)

catkin_package()
include_directories("/usr/include/eigen3")
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  image_transport
  OpenCV
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(AStest src/main.cpp)
add_library(newAStar src/AStar.cpp)
add_library(ASMapCreator src/ASMapCreator.cpp)
target_link_libraries(ASMapCreator newAStar)
target_link_libraries(AStest ${catkin_LIBRARIES} newAStar ASMapCreator)
