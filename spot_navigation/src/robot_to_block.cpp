#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher marker_array_pub_;

void odomCallback (const nav_msgs::Odometry robot_pose)
{
  visualization_msgs::MarkerArray output_marker;

  // Cube
  visualization_msgs::Marker robot_node;
  robot_node.header.frame_id = robot_pose.header.frame_id;
  robot_node.header.stamp = ros::Time::now();
  robot_node.type = visualization_msgs::Marker::CUBE;
  robot_node.id = 1;
  robot_node.action = visualization_msgs::Marker::ADD;
  robot_node.pose.position =  robot_pose.pose.pose.position;
  robot_node.pose.orientation =  robot_pose.pose.pose.orientation;  
  robot_node.color.r = 0.0;
  robot_node.color.g = 0.0;
  robot_node.color.b = 0.0;
  robot_node.color.a = 0.5;
  robot_node.scale.x = 0.4;
  robot_node.scale.y = 0.4;     
  robot_node.scale.z = 0.4;     
  output_marker.markers.push_back(robot_node);

  // Text
  visualization_msgs::Marker robot_type;
  robot_type.header.frame_id = robot_pose.header.frame_id;
  robot_type.header.stamp = ros::Time::now();
  robot_type.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  robot_type.id = 2;
  robot_type.action = visualization_msgs::Marker::ADD;
  robot_type.pose.position.x =  robot_pose.pose.pose.position.x;
  robot_type.pose.position.y =  robot_pose.pose.pose.position.y;
  robot_type.pose.position.z =  robot_pose.pose.pose.position.z + 0.4;
  robot_type.pose.orientation =  robot_pose.pose.pose.orientation;  
  robot_type.color.r = 1.0;
  robot_type.color.g = 1.0;
  robot_type.color.b = 1.0;
  robot_type.color.a = 1.0;  
  robot_type.scale.z = 0.5;     
  robot_type.text = "robot";
  output_marker.markers.push_back(robot_type);

  // Publish the data
  marker_array_pub_.publish(output_marker);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "robot_to_block");
  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe("odom", 1, odomCallback);

  // Create a ROS publisher for the output point cloud
  marker_array_pub_ = n.advertise<visualization_msgs::MarkerArray>("robot_pose", 1);

  // Spin
  ros::spin ();
}

