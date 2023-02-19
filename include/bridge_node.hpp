/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <zmqpp/zmqpp.hpp>
/*
zmqpp is the c++ wrapper around ZeroMQ
Intall zmqpp first:
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html
*/
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
std::string ns; // namespace of this node
XmlRpc::XmlRpcValue ip_xml;
XmlRpc::XmlRpcValue send_topics_xml;
XmlRpc::XmlRpcValue recv_topics_xml;
int len_send; // length(number) of send topics
int len_recv; // length(number) of receive topics

std::map<std::string, std::string> ip_map; // map host name and IP

std::vector<TopicInfo> sendTopics; // send topics info struct vector
std::vector<TopicInfo> recvTopics; // receive topics info struct vector

// ********************* zmq socket initialize ***************************
zmqpp::context_t context;
std::vector<std::unique_ptr<zmqpp::socket>> senders;   //index senders
std::vector<std::unique_ptr<zmqpp::socket>> receivers; //index receivers

// ******************* ROS subscribe and publish *************************
std::vector<ros::Subscriber> topic_subs;
std::vector<ros::Publisher> topic_pubs;

// ******************* send frequency control ***************************
std::vector<ros::Time> sub_t_last;
std::vector<int> send_num;
bool send_freq_control(int i);

// ****************** launch receive threads *****************************
std::vector<bool> recv_thread_flags;
std::vector<bool> recv_flags_last;
std::vector<std::thread> recv_threads;
void recv_func(int i);

// ***************** stop send/receive ******************************
void stop_send(int i);
void stop_recv(int i);

#endif