#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <ros/console.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <vector>

ros::Publisher spot_pub, haetae_pub;
sensor_msgs::PointCloud2 spot_ret_cloud, haetae_ret_cloud;
tf::StampedTransform spot_transform, haetae_transform;

// to add Haetae's position at SPOT's lidar-pointcloud
void spot_pointcloud_add (sensor_msgs::PointCloud2 cloud_pc) {
    pcl::PointCloud<pcl::PointXYZ> cloud_XYZ;
    pcl::fromROSMsg(cloud_pc, cloud_XYZ);
    pcl::PointXYZ point_xyz_1, point_xyz_2, point_xyz_3, point_xyz_4; 

    tf::Matrix3x3 spot_rotation(spot_transform.getRotation());
    tf::Matrix3x3 haetae_rotation(haetae_transform.getRotation());

    // 4 vectors representing haetae's body outline
    tf::Vector3 haetae_body_rot_1 = {0.3, 0.3, 0.0}; 
    tf::Vector3 haetae_body_rot_2 = {0.3, -0.3, 0.0};
    tf::Vector3 haetae_body_rot_3 = {-0.3, 0.3, 0.0};
    tf::Vector3 haetae_body_rot_4 = {-0.3, -0.3, 0.0};
    
    // (Haetae's position) - (SPOT's position) : vector which represents Haetae's position according to the SPOT frame
    tf::Vector3 a_1={haetae_transform.getOrigin().x()-(spot_transform.getOrigin().x()+1.5), \
    haetae_transform.getOrigin().y()-(spot_transform.getOrigin().y()-1.5), 0.0};
    tf::Vector3 a_2={haetae_transform.getOrigin().x()-(spot_transform.getOrigin().x()+1.5), \
    haetae_transform.getOrigin().y()-(spot_transform.getOrigin().y()-1.5), 0.0};
    tf::Vector3 a_3={haetae_transform.getOrigin().x()-(spot_transform.getOrigin().x()+1.5), \
    haetae_transform.getOrigin().y()-(spot_transform.getOrigin().y()-1.5), 0.0};
    tf::Vector3 a_4={haetae_transform.getOrigin().x()-(spot_transform.getOrigin().x()+1.5), \
    haetae_transform.getOrigin().y()-(spot_transform.getOrigin().y()-1.5), 0.0};

    tf::Vector3 point_xyz_vec_1 = spot_rotation.transpose()*(a_1 + haetae_rotation*haetae_body_rot_1);
    tf::Vector3 point_xyz_vec_2 = spot_rotation.transpose()*(a_2 + haetae_rotation*haetae_body_rot_2);
    tf::Vector3 point_xyz_vec_3 = spot_rotation.transpose()*(a_3 + haetae_rotation*haetae_body_rot_3);
    tf::Vector3 point_xyz_vec_4 = spot_rotation.transpose()*(a_4 + haetae_rotation*haetae_body_rot_4);

    point_xyz_1.x=point_xyz_vec_1.x();
    point_xyz_1.y=point_xyz_vec_1.y();
    point_xyz_1.z=1.0;

    point_xyz_2.x=point_xyz_vec_2.x();
    point_xyz_2.y=point_xyz_vec_2.y();
    point_xyz_2.z=1.0;

    point_xyz_3.x=point_xyz_vec_3.x();
    point_xyz_3.y=point_xyz_vec_3.y();
    point_xyz_3.z=1.0;

    point_xyz_4.x=point_xyz_vec_4.x();
    point_xyz_4.y=point_xyz_vec_4.y();
    point_xyz_4.z=1.0;
    
    cloud_XYZ.push_back(point_xyz_1);
    cloud_XYZ.push_back(point_xyz_2);
    cloud_XYZ.push_back(point_xyz_3);
    cloud_XYZ.push_back(point_xyz_4);

    pcl::toROSMsg(cloud_XYZ, spot_ret_cloud);
    spot_ret_cloud.header.frame_id = cloud_pc.header.frame_id; 
}


// to add SPOT's position at Haetae's lidar-pointcloud
void haetae_pointcloud_add (sensor_msgs::PointCloud2 cloud_pc) {
    pcl::PointCloud<pcl::PointXYZ> cloud_XYZ;
    pcl::fromROSMsg(cloud_pc, cloud_XYZ);
    pcl::PointXYZ point_xyz_1, point_xyz_2, point_xyz_3, point_xyz_4; 

    tf::Matrix3x3 haetae_rotation(haetae_transform.getRotation());
    tf::Matrix3x3 spot_rotation(spot_transform.getRotation());

    // 4 vectors representing SPOT's body outline
    tf::Vector3 spot_body_rot_1 = {0.4, 0.2, 0.0}; 
    tf::Vector3 spot_body_rot_2 = {0.4, -0.2, 0.0};
    tf::Vector3 spot_body_rot_3 = {-0.4, 0.2, 0.0};
    tf::Vector3 spot_body_rot_4 = {-0.4, -0.2, 0.0};

    // (SPOT's position) - (Haetae's position) : vector which represents SPOT's position according to the Haetae frame
    tf::Vector3 a_1={(spot_transform.getOrigin().x()+1.5)-haetae_transform.getOrigin().x(), \
    (spot_transform.getOrigin().y()-1.5)-haetae_transform.getOrigin().y(), 0.0};
    tf::Vector3 a_2={(spot_transform.getOrigin().x()+1.5)-haetae_transform.getOrigin().x(), \
    (spot_transform.getOrigin().y()-1.5)-haetae_transform.getOrigin().y(), 0.0};
    tf::Vector3 a_3={(spot_transform.getOrigin().x()+1.5)-haetae_transform.getOrigin().x(), \
    (spot_transform.getOrigin().y()-1.5)-haetae_transform.getOrigin().y(), 0.0};
    tf::Vector3 a_4={(spot_transform.getOrigin().x()+1.5)-haetae_transform.getOrigin().x(), \
    (spot_transform.getOrigin().y()-1.5)-haetae_transform.getOrigin().y(), 0.0};

    tf::Vector3 point_xyz_vec_1 = haetae_rotation.transpose()*(a_1 + spot_rotation*spot_body_rot_1);
    tf::Vector3 point_xyz_vec_2 = haetae_rotation.transpose()*(a_2 + spot_rotation*spot_body_rot_2); 
    tf::Vector3 point_xyz_vec_3 = haetae_rotation.transpose()*(a_3 + spot_rotation*spot_body_rot_3); 
    tf::Vector3 point_xyz_vec_4 = haetae_rotation.transpose()*(a_4 + spot_rotation*spot_body_rot_4); 

    point_xyz_1.x=point_xyz_vec_1.x();
    point_xyz_1.y=point_xyz_vec_1.y();
    point_xyz_1.z=-0.6;

    point_xyz_2.x=point_xyz_vec_2.x();
    point_xyz_2.y=point_xyz_vec_2.y();
    point_xyz_2.z=-0.6;

    point_xyz_3.x=point_xyz_vec_3.x();
    point_xyz_3.y=point_xyz_vec_3.y();
    point_xyz_3.z=-0.6;

    point_xyz_4.x=point_xyz_vec_4.x();
    point_xyz_4.y=point_xyz_vec_4.y();
    point_xyz_4.z=-0.6;

    cloud_XYZ.push_back(point_xyz_1);
    cloud_XYZ.push_back(point_xyz_2);
    cloud_XYZ.push_back(point_xyz_3);
    cloud_XYZ.push_back(point_xyz_4);

    pcl::toROSMsg(cloud_XYZ, haetae_ret_cloud);
    haetae_ret_cloud.header.frame_id = cloud_pc.header.frame_id; 
}

int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init(argc, argv, "add_point");
    ros::NodeHandle n;
  
    tf::TransformListener spot_listener, haetae_listener;

    ros::Subscriber sub = n.subscribe("cloud_registered_body2", 1, spot_pointcloud_add);
    ros::Subscriber sub2 = n.subscribe("/haetae/velodyne_points", 1, haetae_pointcloud_add);
    ros::Rate rate(10);

    spot_pub = n.advertise<sensor_msgs::PointCloud2>("cloud_registered_body3", 1);
    // need to remap "cloud_registered_body2" to "cloud_registered_body3" at pointcloud to laser node
    haetae_pub = n.advertise<sensor_msgs::PointCloud2>("/haetae/velodyne_points2", 1);
    // need to remap "/haetae/velodyne_points" to "/haetae/velodyne_points2" at pointcloud to laser node
    while(n.ok()) {
        try{
            spot_listener.lookupTransform("spot/2d_map", "spot/base_link", ros::Time(0), spot_transform);
            ROS_INFO("Got a SPOT transform! x = %f, y = %f",spot_transform.getOrigin().x(), spot_transform.getOrigin().y());
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("SPOT transform listen Error! %s", ex.what());
        }

        try {
            haetae_listener.lookupTransform("haetae_map", "haetae_base_link", ros::Time(0), haetae_transform);
            ROS_INFO("Got a Haetae transform! x = %f, y = %f",haetae_transform.getOrigin().x(), haetae_transform.getOrigin().y());
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("Haetae transform listen Error! %s", ex.what());
        }
        spot_pub.publish(spot_ret_cloud);
        haetae_pub.publish(haetae_ret_cloud);
        ros::spinOnce();
        rate.sleep();
    }
}