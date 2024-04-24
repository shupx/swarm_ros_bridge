cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bridge_node src/bridge_node.cpp)

target_link_libraries(bridge_node
   ${catkin_LIBRARIES}
   zmq
   zmqpp
)


install(TARGETS bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# scripts
install(PROGRAMS
	scripts/listener.py
	scripts/talker.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
