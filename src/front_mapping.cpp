#include "depth_combine/front_mapping.h"

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "front_mapping");
  FrontMapping fm;
  ros::spin ();
}

FrontMapping::FrontMapping() : nh("front_mapping")
{
  msg_sub = nh.subscribe<sensor_msgs::PointCloud2>("/forward_pelvis_realsense_d430/depth/color/points", 1, &FrontMapping::FrontCB, this);

  msg_pub = nh.advertise<sensor_msgs::PointCloud2>("front_filtered_std", 1);
}

template <typename T>
void FrontMapping::FrontCB(const sensor_msgs::PointCloud2::ConstPtr &forward)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_front = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr_front(cloud_front);
  pcl::PCLPointCloud2 cloud_filtered_front;

  // Convert to PCL data type
  pcl_conversions::toPCL(*forward, *cloud_front);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_f;
  sor_f.setInputCloud (cloudPtr_front);
  sor_f.setLeafSize (0.05, 0.05, 0.05);
  sor_f.filter (cloud_filtered_front);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 front_filtered_std;
  pcl_conversions::moveFromPCL(cloud_filtered_front, front_filtered_std);

  // Publish the data
  msg_pub.publish (front_filtered_std);
}