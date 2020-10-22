#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

ros::Publisher pub;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tf2_listener;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& forward , const sensor_msgs::PointCloud2ConstPtr& downward, const sensor_msgs::PointCloud2ConstPtr& backward)
{
  //Get tf from world to front_depth
  geometry_msgs::TransformStamped transform_stamped_front = tfBuffer.lookupTransform("forward_pelvis_realsense_d430_depth_frame","world", 
        ros::Time(0));
  
  geometry_msgs::TransformStamped transform_stamped_down = tfBuffer.lookupTransform("downward_pelvis_realsense_d430_depth_frame","world", 
        ros::Time(0));
  
  geometry_msgs::TransformStamped transform_stamped_back = tfBuffer.lookupTransform("backward_pelvis_realsense_d430_depth_frame","world", 
        ros::Time(0));
  
  //Container for converted cloud
  sensor_msgs::PointCloud2 conv_cloud_front;
  sensor_msgs::PointCloud2 conv_cloud_down;
  sensor_msgs::PointCloud2 conv_cloud_back;

  //Convert coordinate using pcl_ros::transformPointCloud
  Eigen::Matrix4f mat_front = tf2::transformToEigen(transform_stamped_front.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat_front, *forward, conv_cloud_front);
  
  Eigen::Matrix4f mat_down = tf2::transformToEigen(transform_stamped_down.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat_down, *downward, conv_cloud_down);
  
  Eigen::Matrix4f mat_back = tf2::transformToEigen(transform_stamped_back.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat_back, *backward, conv_cloud_back);
  
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_front = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr_front(cloud_front);
  pcl::PCLPointCloud2 cloud_filtered_front;
  
  pcl::PCLPointCloud2* cloud_down = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr_down(cloud_down);
  pcl::PCLPointCloud2 cloud_filtered_down;
  
  pcl::PCLPointCloud2* cloud_back = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr_back(cloud_back);
  pcl::PCLPointCloud2 cloud_filtered_back;

  // Convert to PCL data type
  pcl_conversions::toPCL(conv_cloud_front, *cloud_front);

  pcl_conversions::toPCL(conv_cloud_down, *cloud_down);

  pcl_conversions::toPCL(conv_cloud_back, *cloud_back);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_f;
  sor_f.setInputCloud (cloudPtr_front);
  sor_f.setLeafSize (0.05, 0.05, 0.05);
  sor_f.filter (cloud_filtered_front);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_d;
  sor_d.setInputCloud (cloudPtr_down);
  sor_d.setLeafSize (0.05, 0.05, 0.05);
  sor_d.filter (cloud_filtered_down);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_b;
  sor_b.setInputCloud (cloudPtr_back);
  sor_b.setLeafSize (0.05, 0.05, 0.05);
  sor_b.filter (cloud_filtered_back);


  //Combine clouds
  pcl::PCLPointCloud2 cloud_combined; 
  pcl::concatenatePointCloud (cloud_combined, cloud_filtered_front, cloud_combined);
  pcl::concatenatePointCloud (cloud_combined, cloud_filtered_down, cloud_combined);
  pcl::concatenatePointCloud (cloud_combined, cloud_filtered_back, cloud_combined);
  
  // Convert to ROS data type
  sensor_msgs::PointCloud2 depth_combined;
  pcl_conversions::moveFromPCL(cloud_combined, depth_combined);

  // Publish the data
  pub.publish (depth_combined);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  //Synchronize the message before calling cloud_cb
  message_filters::Subscriber<sensor_msgs::PointCloud2> forward_sub(nh, "forward_pelvis_realsense_d430/depth/color/points", 1000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> downward_sub(nh, "downward_pelvis_realsense_d430/depth/color/points", 1000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> backward_sub(nh, "backward_pelvis_realsense_d430/depth/color/points", 1000);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), forward_sub, downward_sub, backward_sub);
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2, _3));

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("depth_combined", 1);

  //tf2 listener
  tf2_listener = new tf2_ros::TransformListener(tfBuffer);

  // Spin
  ros::spin ();
}