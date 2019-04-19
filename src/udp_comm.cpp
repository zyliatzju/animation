#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"

#include <iostream>
#include <thread>
#include <stdio.h>
#include <time.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include "animation/cassie_sm_t.hpp"
#include "animation/udp_proto.hpp"

// Connect user computer to target computer
int sock = 0;

// Create packet input/output buffers
unsigned char recvbuf[PACKET_HEADER_LEN + CASSIE_SM_OUT_T_PACKED_LEN];
unsigned char sendbuf[PACKET_HEADER_LEN + CASSIE_SM_IN_T_PACKED_LEN];

// Split header and data
const unsigned char *header_in = recvbuf; //heade has 2 bytes
const unsigned char *data_in = &recvbuf[2];
unsigned char *header_out = sendbuf;
unsigned char *data_out = &sendbuf[2];

// Create cassie input/output structs
cassie_sm_out_t cassie_sm_out;
cassie_sm_in_t cassie_sm_in;

// Create header information struct
packet_header_info_t header_info = {0};

// Create global msg to be overridden
int mid = 0;

std_msgs::Int16MultiArray smach_fb;
ros::Publisher smach_rtc_pub;

// Dummy controller that outputs zeros
static void set_smach_in(cassie_sm_in_t *in
	)
{
  memset(in, 0, sizeof(cassie_sm_in_t));
  in->motion_id = mid;
}


void callbackMotionId(const std_msgs::Int32::ConstPtr& msg)
{
  mid = msg->data;
}


void udpCall()
{
  ROS_INFO("UDP connection established.");
  while (true)
  {
    // Process incoming header and write outgoing header
    process_packet_header(&header_info, header_in, header_out);

    // Run set input for smach 
    //set_smach_in(&cassie_sm_in);
    cassie_sm_in.motion_id= mid;

    // Pack cassie input struct into outgoing packet
    pack_cassie_sm_in_t(&cassie_sm_in, data_out);
    
    // Send response
    int nbites = 0;
    nbites = send_packet(sock, sendbuf, sizeof sendbuf, NULL, 0);
    //ROS_INFO("nbites is %d",nbites);

    usleep(5000);
  }
}


void udpReceive()
{
  ROS_INFO("UDP Receiver connection established.");
  while (true)
  {
    // Poll for a new packet
    wait_for_packet(sock, recvbuf, sizeof recvbuf, NULL, NULL);
	
    // Process incoming header and write outgoing header
    process_packet_header(&header_info, header_in, header_out);

    // Unpack received data into cassie output struct
    unpack_cassie_sm_out_t(data_in, &cassie_sm_out);

    smach_fb.data.clear();
    smach_fb.data.push_back(cassie_sm_out.track_ani);
    smach_fb.data.push_back(cassie_sm_out.in_ani);
    smach_fb.data.push_back(cassie_sm_out.ani_finished);
    smach_rtc_pub.publish(smach_fb);

    usleep(5000);
  }
}

void timerCallback(const ros::TimerEvent& event)
{
	ROS_INFO("[udp] sendbuf: %d, %d, %d, %d, %d, %d",sendbuf[0],sendbuf[1],sendbuf[2],sendbuf[3],sendbuf[4],sendbuf[5]);
	ROS_INFO("[udp] recvbuf: %d, %d, %d, %d, %d, %d, %d, %d",recvbuf[0],recvbuf[1],recvbuf[2],recvbuf[3],recvbuf[4],recvbuf[5],recvbuf[6],recvbuf[7]);
}

void convertParamToSock(ros::NodeHandle *nh, int *sock)
{
  ROS_INFO("Retrieving node UDP parameters.");

  std::string rtc_ip;
  std::string nuc_ip;
  std::string rtc_port;
  std::string nuc_port;

  // Read parameters from the launch file.
  if (!nh->getParam("rtc_ip", rtc_ip))
  {
    ros::param::param<std::string>("rtc_ip", rtc_ip, "10.10.10.3");
  }

  if (!nh->getParam("nuc_ip", nuc_ip))
  {
    ros::param::param<std::string>("nuc_ip", nuc_ip, "10.10.10.100");
  }

  if (!nh->getParam("rtc_port", rtc_port))
  {
    ros::param::param<std::string>("rtc_port", rtc_port, "25000");
  }

  if (!nh->getParam("nuc_port", nuc_port))
  {
    ros::param::param<std::string>("nuc_port", nuc_port, "25001");
  }
  
  ROS_INFO("Set parameter rtc_ip: %s", rtc_ip.c_str());
  ROS_INFO("Set parameter nuc_ip: %s", nuc_ip.c_str());
  ROS_INFO("Set parameter rtc_port: %s", rtc_port.c_str());
  ROS_INFO("Set parameter nuc_port: %s", nuc_port.c_str());

  // Generate the sock value.
  *sock = udp_init_client(
    rtc_ip.c_str(), rtc_port.c_str(),
    nuc_ip.c_str(), nuc_port.c_str()
  );
}


void initUdpThread()
{
  ROS_INFO("Establishing UDP connection.");
  std::thread udp_call(udpCall);
  udp_call.detach();
  ROS_INFO("UDP thread detached.");
}

void initUdpRevThread()
{
  ROS_INFO("Establishing UDP receiver connection.");
  std::thread udp_call(udpReceive);
  udp_call.detach();
  ROS_INFO("UDP thread detached.");
}


int main(int argc, char **argv)
{
  ROS_INFO("Starting node UDP.");
  ros::init(argc, argv, "udp");
  ros::NodeHandle nh("~");

  convertParamToSock(&nh, &sock);
  initUdpThread();
  initUdpRevThread();
  ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

  ros::Subscriber sub_vel = nh.subscribe("motion_msg", 10, callbackMotionId);
  smach_rtc_pub = nh.advertise<std_msgs::Int16MultiArray>("rtc_feedback", 10);
  ros::Rate r(200);
  ROS_INFO("The UDP node is alive.");

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
