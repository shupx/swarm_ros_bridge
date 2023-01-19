# swarm_ros_bridge

## Introduction

A lightweight middle interface ROS package that enables the specified ROS messages transmission among swarm robots through socket communication.

Compared with ROS1 multi-robot wireless communication, it has the following benefits:

-  **Robust**: No need for base station ROS master launching first. Support each robot launching in a random sequence and connecting each other autonomously.

-  **Flexible**:  You can choose the sending/receiving ROS topics rather than transferring all topics as ROS1 does.

-  **Easy to use**:  Specify all the IP and ROS topics in one configuration file.

Compared with ROS2 DDS communication, it has the following benefits:

-  **Lightweight**: It is a small ROS bridge node subscribing and sending remote ROS topics, so connecting with other ROS1 nodes is easy.

-  **Reliable**: It uses ZeroMQ socket communication based on TCP protocol while ROS2 is based on DDS, whose default protocol is UDP (unreliable). DDS is mainly designed for data exchange between native processes under wired communication rather than remote wireless communication.


## Structure

```bash
└── swarm_ros_bridge
    ├── CMakeLists.txt
    ├── config
    │   └── ros_topics.yaml  # Config file to specify send/receive ROS topics
    ├── include
    │   ├── bridge_node.hpp  # Header file of bridge_node.cpp
    │   ├── ros_sub_pub.hpp  # Header file for different ROS message type.
    ├── launch
    │   └── test.launch
    ├── package.xml
    └── src
        └── bridge_node.cpp  # @brief Reliable TCP bridge for ros data transfer in unstable network.
                             # It will send/receive the specified ROS topics in ../config/ros_topics.yaml
                             # It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
                             # supports 1-N pub-sub connection even with TCP protocol.
```


## Install

```bash
## clone this package
mkdir -p swarm_ros_bridge_ws/src  # or your own ros workspace
cd swarm_ros_bridge_ws/src
git clone https://gitee.com/shu-peixuan/swarm_ros_bridge.git
# or 'git clone https://github.com/shupx/swarm_ros_bridge.git'

## install dependencies
sudo apt install libzmqpp-dev
# or 'rosdep install --from-path swarm_ros_bridge/'

## build
cd ../
catkin_make
source devel/setup.bash
```


## Usage

1. Specify the IP and ROS topic information in `src/swarm_ros_bridge/config/ros_topics.yaml`. 

- For sending topics, IP is self IP (* for example) and port should be different as it binds to the "tcp://*:port". 
- For receiving topics, IP and port should be the remote source IP and port as it connects to the "tcp://srcIP:srcPort".

(The `max_freq` only guarantees the sending frequency is lower than that but not be that. If the send_topics frequency is larger than max_freq, the node will decrease it by 2x, 3x, ... until it satisfies the max_freq.)

2. Launch the bridge_node:

```bash
roslaunch swarm_ros_bridge test.launch
```

3. Publish messages into send_topics and check that remote recv_topics are receiving these messages. The console will also print INFO the first time recv_topics receive messages.


## Advanced

### * More ROS message types

The default supported ROS message types are only `sensor_msgs/Imu` and `geometry_msgs/Twist`. If you need more types:

1. Modify the macros about MSG_TYPEx and MSG_CLASSx in `src/swarm_ros_bridge/include/ros_sub_pub.hpp`, then it will generate template functions for different ros message types.  

```cpp
// In ros_sub_pub.hpp
// uncomment and modify the following lines:
#include <xxx_msgs/yy.h>
#define MSG_TYPE3 "xxx_msgs/yy"
#define MSG_CLASS3 xxx_msgs::yy
```

We support up to 10 types modification. If that is still not enough, then you should modify the `topic_subscriber()`, `topic_publisher()` and `deserialize_publish()` in `src/swarm_ros_bridge/include/ros_sub_pub.hpp` according to their styles.

2. Add the dependent package in find_package() of `src/swarm_ros_bridge/CMakeLists.txt`:

```sh
# in CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  xxx_msgs
)
```

3. recompile:

```bash
cd swarm_ros_bridge_ws/
catkin_make
```

### * More send_topics

We support up to 50 send_topics. Modify the following lines in `src/swarm_ros_bridge/include/ros_sub_pub.hpp` if you need more.

```cpp
// in ros_sub_pub.hpp
# define SUB_MAX 50 // max number of subscriber callbacks
//...
template <typename T>
void (*sub_callbacks[])(const T &)=
{
  sub_cb<T,0>, sub_cb<T,1>, ...
};

```

Then recompile:

```bash
cd swarm_ros_bridge_ws/
catkin_make
```

## Future Work

1.  Dynamic RPC, including dynamic node discovery, online topic change, and ground station monitor.
2.  Support UDP protocol for mass data transmission like video streams. 


## Contributor

Shu Peixuan (shupeixuan@qq.com) 2023.1.1

![img1](pictures/img1.png)
