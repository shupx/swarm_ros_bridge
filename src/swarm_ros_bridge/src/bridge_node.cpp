#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

// using namespace std;

/*
ros::Subscriber imu_sub_;
ros::Publisher cmdvel_pub_;

std::vector<int> id_list_;
std::vector<string> ip_list_;
int self_id_;
int self_id_in_bridge_;
int drone_num_;
int ground_station_num_;
double odom_broadcast_freq_;
bool is_groundstation_;

unique_ptr<ReliableBridge> bridge;

inline int remap_ground_station_id(int id)
{
  return id+drone_num_;
}

template <typename T>
int send_to_all_drone_except_me(string topic, T &msg)
{
  int err_code = 0; 
  for (int i = 0; i < drone_num_; ++i)// Only send to all drones.
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(i,topic,msg);
    if(err_code< 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

template <typename T>
int send_to_all_groundstation_except_me(string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i)// Only send to all groundstations.
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(ind,topic,msg);
    if(err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!",typeid(T).name());
    }
  }
  return err_code;
}

void register_callbak_to_all_groundstation(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < ground_station_num_; ++i)
  {
    int ind = remap_ground_station_id(i);
    if (ind == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(ind,topic_name,callback);
  }
}

void register_callbak_to_all_drones(string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < drone_num_; ++i)
  {
    if (i == self_id_in_bridge_)  //skip myself
    {
      continue;
    }
    bridge->register_callback(i,topic_name,callback);
  }
}
 

// Here is callback from local topic.
void odom_sub_cb(const nav_msgs::OdometryPtr &msg)
{

  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("drone_") + std::to_string(self_id_);

  other_odoms_pub_.publish(msg); // send to myself
  send_to_all_drone_except_me("/odom",*msg);// Only send to drones.
  send_to_all_groundstation_except_me("/odom",*msg);// Only send to ground stations.
}

void object_odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{
  msg->child_frame_id = string("obj_") + std::to_string(self_id_);

  object_odoms_pub_.publish(msg); // send to myself
  send_to_all_drone_except_me("/object_odom",*msg);// Only send to drones.
  send_to_all_groundstation_except_me("/object_odom",*msg);// Only send to ground stations.
}

void one_traj_sub_cb(const traj_utils::MINCOTrajPtr &msg)
{
  one_traj_pub_.publish(msg);  // Send to myself.
  if (bridge->send_msg_to_all("/traj_from_planner",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (ONE_TRAJ)!!!");
  }
}

void joystick_sub_cb(const sensor_msgs::JoyPtr &msg)
{
  joystick_pub_.publish(msg); // Send to myself.
  send_to_all_drone_except_me("/joystick",*msg);
}

void goal_sub_cb(const quadrotor_msgs::GoalSetPtr &msg)
{
  if (msg->drone_id==self_id_in_bridge_)
  {
    goal_pub_.publish(msg);  // Send to myself.
    return;
  }
  
  if(bridge->send_msg_to_one(msg->drone_id,"/goal",*msg)< 0)
  {
    ROS_ERROR("[Bridge] SEND ERROR (GOAL)!!!");
  }
}

void goal_exploration_sub_cb(const quadrotor_msgs::GoalSetPtr &msg)
{
  if (bridge->send_msg_to_all("/goal_exploration",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (goal_exploration)!!!");
  }

}

void star_cvx_sub_cb(const sensor_msgs::PointCloud2Ptr &msg)
{
  if (bridge->send_msg_to_all("/star_cvx",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (star_cvx)!!!");
  }

}

void frontier_sub_cb(const sensor_msgs::PointCloud2Ptr &msg)
{

  if (bridge->send_msg_to_all("/frontier",*msg))
  {
    ROS_ERROR("[Bridge] SEND ERROR (frontier)!!!");
  }

}


// Here is callback when the brodge received the data from others.
void odom_bridge_cb(int ID, ros::SerializedMessage& m)
{
  nav_msgs::Odometry odom_msg_;
  ros::serialization::deserializeMessage(m,odom_msg_);
  other_odoms_pub_.publish(odom_msg_);
}

void object_odom_bridge_cb(int ID, ros::SerializedMessage& m)
{
  nav_msgs::Odometry object_odom_msg_;
  ros::serialization::deserializeMessage(m,object_odom_msg_);
  object_odoms_pub_.publish(object_odom_msg_);
}

void goal_bridge_cb(int ID, ros::SerializedMessage& m)
{
  quadrotor_msgs::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_pub_.publish(goal_msg_);
}

void traj_bridge_cb(int ID, ros::SerializedMessage& m)
{
  traj_utils::MINCOTraj MINCOTraj_msg_;
  ros::serialization::deserializeMessage(m,MINCOTraj_msg_);
  one_traj_pub_.publish(MINCOTraj_msg_);
}

void joystick_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::Joy joystick_msg_;
  ros::serialization::deserializeMessage(m,joystick_msg_);
  joystick_pub_.publish(joystick_msg_);
}

void goal_exploration_bridge_cb(int ID, ros::SerializedMessage& m)
{
  quadrotor_msgs::GoalSet goal_msg_;
  ros::serialization::deserializeMessage(m,goal_msg_);
  goal_exploration_pub_.publish(goal_msg_);
}

void star_cvx_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  star_cvx_pub_.publish(point_msg_);
}

void frontier_bridge_cb(int ID, ros::SerializedMessage& m)
{
  sensor_msgs::PointCloud2 point_msg_;
  ros::serialization::deserializeMessage(m,point_msg_);
  frontier_pub_.publish(point_msg_);
}
*/


// template <typename T>
// void sub_cb(const T &msg)
// {
//   std::cout << msg << std::endl;
//   //std::cout << port << std::endl;
// }

# define SUB_MAX 50 // max number of subscriber callbacks

struct TopicInfo
{
  std::string name;
  std::string type;
  std::string ip;
  int port;
};

template <typename T, int cout>
void sub_cb(const T &msg)
{
  std::cout << msg << std::endl;
  std::cout << cout << std::endl;
}

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


//TODO: generate or delete topic message transfers through a remote zmq service.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  //************************ Parse configuration file **************************
  XmlRpc::XmlRpcValue ip_xml;
  XmlRpc::XmlRpcValue send_topics_xml;
  XmlRpc::XmlRpcValue recv_topics_xml;
  int len_send; // length(number) of send topics
  int len_recv; // length(number) of receive topics

  // get hostnames and IPs
  if (nh.getParam("IP", ip_xml) == false){
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return 1;
  }
  // get "send topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("send_topics", send_topics_xml)){
    ROS_ASSERT(send_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_send = send_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No send_topics found in the configuration!");
    len_send = 0;
  }
  // get "receive topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("recv_topics", recv_topics_xml)){
    ROS_ASSERT(recv_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_recv = recv_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No recv_topics found in the configuration!");
    len_recv = 0;
  }

  if (len_send > SUB_MAX)
  {
    ROS_FATAL("[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }

  std::map<std::string, std::string> ip_map; // map host name and IP
  for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter)
  {
    std::string host_name = iter->first;
    std::string host_ip = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end())
    { // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    ip_map[host_name] = host_ip;
  }

  std::vector<TopicInfo> sendTopics;
  std::vector<TopicInfo> recvTopics;

  std::set<int> srcPorts; // for duplicate check 
  std::cout << "send: " << std::endl;
  for (int32_t i=0; i < len_send; ++i)
  {
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    std::string srcIP = ip_map[send_topic_xml["srcIP"]];
    int srcPort = send_topic_xml["srcPort"];
    TopicInfo topic = {topic_name, msg_type, srcIP, srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    std::cout << topic.name << std::endl;
  }

  std::cout << "receive: " << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    std::string srcIP = ip_map[recv_topic_xml["srcIP"]];
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {topic_name, msg_type, srcIP, srcPort};
    recvTopics.emplace_back(topic);
    std::cout << topic.name << std::endl;
  }

  // ******************* ROS subscribe and publish *************************
  std::vector<ros::Subscriber> topic_subs(len_send);
  std::vector<ros::Publisher> topic_pubs(len_recv);

  for (int32_t i=0; i < len_send; ++i) // ROS topic subsrcibe and send
  {
    //nh_sub<type_name>(sendTopics[i].name, nh, i);
    topic_subs[i] = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh, i);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  for (int32_t i=0; i < len_recv; ++i) // ROS topic receive and publish
  {
    //topic_pubs[i] = nh.advertise<geometry_msgs::Twist>(recvTopics[i].name, 10);
    topic_pubs[i] = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh);
  }

  ros::spin();

  return 0;
}
