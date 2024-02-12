#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void mapcloudCallback (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 output;

  // Create filter
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.3, 10.0);
  pass.filter (output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "lidar_filtering");
  ros::NodeHandle n;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = n.subscribe("/ouster/points", 1, mapcloudCallback);

  // Create a ROS publisher for the output point cloud
  // Changed from laser_map to lidar points
  pub = n.advertise<pcl::PCLPointCloud2>("/ouster/filtered_points", 1);

  // Spin
  ros::spin ();
}

