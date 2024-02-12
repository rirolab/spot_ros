#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
// for udp
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

ros::Publisher pub;

// jog
double pad_lx = 0;
double pad_ly = 0;
double pad_rx = 0;
double pad_ry = 0;

int last_status = 0;

// for udp
int sockfd = -1;
bool move = false;
bool stop = false;
int wait_thres = 9;
struct sockaddr_in servaddr;

void cmd_velCallback (const geometry_msgs::Twist::ConstPtr& input)
{
  std::cout << input << std::endl;	
  pad_ly = -input->linear.x;
  pad_lx = -input->linear.y;
  pad_rx = -input->angular.z; 
}

void movebase_state_Callback (const actionlib_msgs::GoalStatusArray::ConstPtr& input)
{
  std::cout << input << std::endl;	
  if (!input->status_list.empty()){
    if (input->status_list.back().status != last_status){
      if (input->status_list.back().status == 1) move = true;
      else stop = true;
    }
    last_status = input->status_list.back().status;       
  }  
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rbq_controller");
  ros::NodeHandle n;

  // udp setting
  if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
      ROS_WARN("socket failed!\n");
  }

  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(28224); // set robot port
  // QString ipaddr = "192.168.0.10"; // set robot ip
  ::inet_pton(AF_INET, "192.168.1.10", &(servaddr.sin_addr));

  // Create a ROS subscriber for cmd_vel
  ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel/smooth", 1, cmd_velCallback); // or topic /cmd_vel
  ros::Subscriber sub_move_state = n.subscribe("/move_base/status", 1, movebase_state_Callback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();

    // If no change
    if (!move && !stop && pad_lx == -0 && pad_ly == -0 && pad_rx == -0){
      loop_rate.sleep();
      continue;
    }

    // make control data
    u_char lx = (pad_lx+1.0)*100;
    u_char ly = (pad_ly+1.0)*100;

    u_char rx = (pad_rx+1.0)*100;
    u_char ry = (pad_ry+1.0)*100;

    u_char l2 = 0;
    u_char r2 = 0;

    u_char bt0 = 0;
    u_char bt1 = 0;

    // Start Move
    if (move){     
      rx = (0.0+1.0)*100;
      lx = (0.0+1.0)*100;
      ly = (0.0+1.0)*100;
      l2 = (1.0+1.0)*100; // LT
      bt0 = bt0|1<<0; // A      
    }

    // Stop
    if (stop){
      rx = (0.0+1.0)*100;
      lx = (0.0+1.0)*100;
      ly = (0.0+1.0)*100;      
      l2 = (1.0+1.0)*100; // LT
      bt0 = bt0|1<<1; // B  
    } 
    
    // ROS_WARN_STREAM(lx);
    // ROS_WARN_STREAM(ly);
    // ROS_WARN_STREAM(rx);

    /*
    u_char lx = (pad->axisLeftX()+1.0)*100;
    u_char ly = (pad->axisLeftY()+1.0)*100;

    u_char rx = (pad->axisRightX()+1.0)*100;
    u_char ry = (pad->axisRightY()+1.0)*100;

    u_char l2 = (pad->buttonL2()+1.0)*100;
    u_char r2 = (pad->buttonR2()+1.0)*100;

    u_char bt0 = 0;
    bt0 = bt0|(pad->buttonA()?1:0)<<0;
    bt0 = bt0|(pad->buttonB()?1:0)<<1;
    bt0 = bt0|(pad->buttonX()?1:0)<<2;
    bt0 = bt0|(pad->buttonY()?1:0)<<3;
    bt0 = bt0|(pad->buttonUp()?1:0)<<4;
    bt0 = bt0|(pad->buttonDown()?1:0)<<5;
    bt0 = bt0|(pad->buttonLeft()?1:0)<<6;
    bt0 = bt0|(pad->buttonRight()?1:0)<<7;

    u_char bt1 = 0;
    bt1 = bt1|(pad->buttonL1()?1:0)<<0;
    bt1 = bt1|(pad->buttonL3()?1:0)<<1;
    bt1 = bt1|(pad->buttonR1()?1:0)<<2;
    bt1 = bt1|(pad->buttonR3()?1:0)<<3;
    bt1 = bt1|(pad->buttonCenter()?1:0)<<4;
    bt1 = bt1|(pad->buttonGuide()?1:0)<<5;
    bt1 = bt1|(pad->buttonStart()?1:0)<<6;
    bt1 = bt1|(pad->buttonSelect()?1:0)<<7;
    */

    // make packet
    u_char msg[12];

    // header
    msg[0] = 0xFF;
    msg[1] = 0xFE;

    // body
    memcpy(&msg[2], &lx, 1);
    memcpy(&msg[3], &ly, 1);
    memcpy(&msg[4], &rx, 1);
    memcpy(&msg[5], &ry, 1);
    memcpy(&msg[6], &l2, 1);
    memcpy(&msg[7], &r2, 1);
    memcpy(&msg[8], &bt0, 1);
    memcpy(&msg[9], &bt1, 1);

    // tail
    msg[10] = 0x00;
    msg[11] = 0x01;

    // send to udp
    if (move || stop){
      for (int i = 0; i < wait_thres; i++){
        ::sendto(sockfd, msg, 12, MSG_CONFIRM, (const struct sockaddr*)&servaddr, sizeof(servaddr));  
        loop_rate.sleep();
      }
      move = false;
      stop = false;
    }
    ::sendto(sockfd, msg, 12, MSG_CONFIRM, (const struct sockaddr*)&servaddr, sizeof(servaddr));  
    loop_rate.sleep();

  }
}
