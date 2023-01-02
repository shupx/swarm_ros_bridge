#ifndef __ROS_SUB_PUB__
#define __ROS_SUB_PUB__
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>


# define SUB_MAX 50 // max number of subscriber callbacks

template <typename T, int i>
void sub_cb(const T &msg);

template <typename T>
void (*sub_callbacks[])(const T &);

template <typename T>
ros::Subscriber nh_sub(std::string topic_name, ros::NodeHandle nh, int i);

ros::Subscriber topic_subscriber(std::string topic_name, std::string msg_type, ros::NodeHandle nh, int i);

ros::Publisher topic_publisher(std::string topic_name, std::string msg_type, ros::NodeHandle nh);


template <typename T>
void (*sub_callbacks[])(const T &)=
{
  sub_cb<T,0>, sub_cb<T,1>, sub_cb<T,2>, sub_cb<T,3>, sub_cb<T,4>,
  sub_cb<T,5>, sub_cb<T,6>, sub_cb<T,7>, sub_cb<T,8>, sub_cb<T,9>,
  sub_cb<T,10>, sub_cb<T,11>, sub_cb<T,12>, sub_cb<T,13>, sub_cb<T,14>,
  sub_cb<T,15>, sub_cb<T,16>, sub_cb<T,17>, sub_cb<T,18>, sub_cb<T,19>,
  sub_cb<T,20>, sub_cb<T,21>, sub_cb<T,22>, sub_cb<T,23>, sub_cb<T,24>,
  sub_cb<T,25>, sub_cb<T,26>, sub_cb<T,27>, sub_cb<T,28>, sub_cb<T,29>,
  sub_cb<T,30>, sub_cb<T,31>, sub_cb<T,32>, sub_cb<T,33>, sub_cb<T,34>,
  sub_cb<T,35>, sub_cb<T,36>, sub_cb<T,37>, sub_cb<T,38>, sub_cb<T,39>,
  sub_cb<T,40>, sub_cb<T,41>, sub_cb<T,42>, sub_cb<T,43>, sub_cb<T,44>,
  sub_cb<T,45>, sub_cb<T,46>, sub_cb<T,47>, sub_cb<T,48>, sub_cb<T,49>
};

template <typename T>
ros::Subscriber nh_sub(std::string topic_name, ros::NodeHandle nh, int i)
{
  return nh.subscribe(topic_name, 10, sub_callbacks<T>[i], ros::TransportHints().tcpNoDelay());
}

ros::Subscriber topic_subscriber(std::string topic_name, std::string msg_type, ros::NodeHandle nh, int i)
{
  if (msg_type == "sensor_msgs/Imu")
    return nh_sub<sensor_msgs::Imu>(topic_name, nh, i);
  else if (msg_type == "geometry_msgs/Twist")
    return nh_sub<geometry_msgs::Twist>(topic_name, nh, i);
  else{
    ROS_FATAL("Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
    exit(1);}
}

ros::Publisher topic_publisher(std::string topic_name, std::string msg_type, ros::NodeHandle nh)
{
  if (msg_type == "sensor_msgs/Imu")
    return nh.advertise<sensor_msgs::Imu>(topic_name, 10);
  else if (msg_type == "geometry_msgs/Twist")
    return nh.advertise<geometry_msgs::Twist>(topic_name, 10);
  else{
    ROS_FATAL("Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
    exit(1);}
}

#endif