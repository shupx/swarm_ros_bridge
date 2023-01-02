#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include "ros_sub_pub.hpp"

struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  std::string ip;
  int port;
};

//********************* Parse configuration file **************************
XmlRpc::XmlRpcValue ip_xml;
XmlRpc::XmlRpcValue send_topics_xml;
XmlRpc::XmlRpcValue recv_topics_xml;
int len_send; // length(number) of send topics
int len_recv; // length(number) of receive topics

std::map<std::string, std::string> ip_map; // map host name and IP

std::vector<TopicInfo> sendTopics; // send topics info struct vector
std::vector<TopicInfo> recvTopics; // receive topics info struct vector

// ******************* ROS subscribe and publish *************************
std::vector<ros::Subscriber> topic_subs;
std::vector<ros::Publisher> topic_pubs;

#endif