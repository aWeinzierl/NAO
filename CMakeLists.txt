cmake_minimum_required(VERSION 2.8.3)
project(nao_HCR_tutorial_6)

add_definitions(-std=c++11)
find_package(catkin REQUIRED cmake_modules COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  actionlib_msgs
  actionlib
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nao_2 src/main.cpp src/NaoControl.cpp src/NaoControl.h)
target_link_libraries(nao_2 ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(nao_2 ${Boost_LIBRARIES})
