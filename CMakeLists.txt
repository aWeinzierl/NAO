cmake_minimum_required(VERSION 2.8.3)
project(nao)

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

add_executable(nao src/main.cpp src/NAO/NaoControl.cpp src/NAO/NaoControl.h src/NAO/Joints.h src/NAO/ContinuousJointSpecification.cpp src/NAO/ContinuousJointSpecification.h src/NAO/DiscreteJointSpecification.cpp src/NAO/DiscreteJointSpecification.h src/NAO/Interval.cpp src/NAO/Interval.h src/StdExtension/MakeUnique.h)
target_link_libraries(nao ${catkin_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(nao ${Boost_LIBRARIES})
