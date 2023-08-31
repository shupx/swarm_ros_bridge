cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rospy
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

FILE(GLOB SRCS src/*.cpp)

foreach(src ${SRCS})
  
get_filename_component(name ${src} NAME_WE)
add_executable(${name} ${src})

target_link_libraries(${name}
   ${catkin_LIBRARIES}
   zmq
   zmqpp
)

endforeach(src ${SRCS})

