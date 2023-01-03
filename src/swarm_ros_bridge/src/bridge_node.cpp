#include "bridge_node.hpp"

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



/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void sub_cb(const T &msg)
{
  // frequency control 
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * sendTopics[i].max_freq < 1.0)
    {return;}
  t_last = t_now;

  // serialize the sending messages into send_buffer
  namespace ser = ros::serialization;
  size_t data_len = ser::serializationLength(msg); // bytes length of msg
  std::unique_ptr<uint8_t> send_buffer(new uint8_t[data_len]);  // create a dynamic length array
  ser::OStream stream(send_buffer.get(), data_len);
  ser::serialize(stream, msg);

  // zmq send message
  zmqpp::message send_array;
  send_array << data_len; 
  /* 
  send_array.add_raw(reinterpret_cast<void const*>(&data_len), sizeof(size_t));
  */
  send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);
  // std::cout << "ready send!" << std::endl;
  senders[i]->send(send_array, false);  //block here, wait for sending
  // std::cout << "send!" << std::endl;

  // std::cout << msg << std::endl;
  // std::cout << i << std::endl;
}


/* uniform deserialize and publish the receiving messages */
template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  T msg;
  // deserialize the receiving messages into ROS msg
  namespace ser = ros::serialization;
  ser::IStream stream(buffer_ptr, msg_size);
  ser::deserialize(stream, msg);
  // publish ROS msg
  topic_pubs[i].publish(msg);
}


/* receive thread function to receive messages and publish them */
void recv_func(int i)
{
  while(true)
  {
    zmqpp::message recv_array;
    // std::cout << "ready receive!" << std::endl;
    if (receivers[i]->receive(recv_array, false))
    {
      // std::cout << "receive!" << std::endl;
      size_t data_len;
      recv_array >> data_len; // unpack meta data
      /*  recv_array.get(&data_len, recv_array.read_cursor++); 
          void get(T &value, size_t const cursor){
            uint8_t const* byte = static_cast<uint8_t const*>(raw_data(cursor)); 
            b = *byte;} */
      // a dynamic length array by unique_ptr
      std::unique_ptr<uint8_t> recv_buffer(new uint8_t[data_len]);  
      // continue to copy the raw_data of recv_array into buffer
      memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
      deserialize_publish(recv_buffer.get(), data_len, recvTopics[i].type, i);

      // std::cout << data_len << std::endl;
      // std::cout << recv_buffer.get() << std::endl;
    }
  }
}


//TODO: generate or delete topic message transfers through a remote zmq service.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  //************************ Parse configuration file **************************
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

  std::cout << "-------------IP------------" << std::endl;
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

  std::cout << "--------send topics--------" << std::endl;
  std::set<int> srcPorts; // for duplicate check 
  for (int32_t i=0; i < len_send; ++i)
  {
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    int max_freq = send_topic_xml["max_freq"];
    std::string srcIP = ip_map[send_topic_xml["srcIP"]];
    int srcPort = send_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    int max_freq = recv_topic_xml["max_freq"];
    std::string srcIP = ip_map[recv_topic_xml["srcIP"]];
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    std::cout << topic.name << std::endl;
  }

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->bind(url);
    senders.emplace_back(std::move(sender)); //sender is now released by std::move
  }

  // receive sockets (zmq socket SUB mode) and recv threads
  for (int32_t i=0; i < len_recv; ++i)
  {
    const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
    std::string const zmq_topic = ""; // "" means all zmq topic
    std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::sub));
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers.emplace_back(std::move(receiver));
  }


  // ******************* ROS subscribe and publish *************************
  //ROS topic subsrcibe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    //nh_sub<type_name>(sendTopics[i].name, nh, i);
    ros::Subscriber subscriber;
    // The uniform callback function is sub_cb()
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh, i);
    topic_subs.emplace_back(subscriber);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    //topic_pubs[i] = nh.advertise<geometry_msgs::Twist>(recvTopics[i].name, 10);
    ros::Publisher publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh);
    topic_pubs.emplace_back(publisher);
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_threads.emplace_back(std::thread(&recv_func, i));
  }

  ros::spin();

  return 0;
}
