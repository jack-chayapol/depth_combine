#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <math.h>

using namespace message_filters;

ros::Publisher pub;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tf2_listener;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& forward , const sensor_msgs::PointCloud2ConstPtr& downward, const sensor_msgs::PointCloud2ConstPtr& backward)
{
 
  //Container for converted cloud
  sensor_msgs::PointCloud2 conv_cloud_front;
  sensor_msgs::PointCloud2 conv_cloud_down;
  sensor_msgs::PointCloud2 conv_cloud_back;


  //Transform Pointcloud
  try 
    {
        pcl_ros::transformPointCloud ("base_link", *forward, conv_cloud_front, tfBuffer);
        pcl_ros::transformPointCloud ("base_link", *downward, conv_cloud_down, tfBuffer);
        pcl_ros::transformPointCloud ("base_link", *backward, conv_cloud_back, tfBuffer);
      
      // ROS_INFO("tf received\n");
    }
  catch (tf2::TransformException &ex) 
    {
      // ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }

  
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

  // Perform the voxle grid filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_f;
  sor_f.setInputCloud (cloudPtr_front);
  sor_f.setLeafSize (0.03, 0.03, 0.03);
  sor_f.filter (cloud_filtered_front);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_d;
  sor_d.setInputCloud (cloudPtr_down);
  sor_d.setLeafSize (0.03, 0.03, 0.03);
  sor_d.filter (cloud_filtered_down);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor_b;
  sor_b.setInputCloud (cloudPtr_back);
  sor_b.setLeafSize (0.03, 0.03, 0.03);
  sor_b.filter (cloud_filtered_back);

  //Combine clouds
  pcl::PCLPointCloud2* cloud_combined = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr_combined(cloud_combined);
  pcl::concatenatePointCloud (*cloud_combined, cloud_filtered_front, *cloud_combined);
  pcl::concatenatePointCloud (*cloud_combined, cloud_filtered_down, *cloud_combined);
  pcl::concatenatePointCloud (*cloud_combined, cloud_filtered_back, *cloud_combined);

  // Container for pcl objects
  pcl::PointCloud<pcl::PointXYZ> *xyz_combined_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCombinedCloudPtr(xyz_combined_cloud);
  pcl::PointCloud<pcl::PointXYZ> *xyz_filtered_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzFilteredCloudPtr(xyz_filtered_cloud);

  // Convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(*cloud_combined, *xyzCombinedCloudPtr);

  // Extracting unwanted points
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*xyzCombinedCloudPtr).size(); i++)
  {
    pcl::PointXYZ pt(xyzCombinedCloudPtr->points[i].x, xyzCombinedCloudPtr->points[i].y, xyzCombinedCloudPtr->points[i].z);
    float min_radius = 1.05;
    float max_radius = 2.7;
    if (sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) < min_radius || sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) > max_radius)
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(xyzCombinedCloudPtr);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*xyzFilteredCloudPtr);

  // Convert to pcl::PCLPointCloud2
  pcl::PCLPointCloud2 filtered_cloud;
  pcl::toPCLPointCloud2( *xyzFilteredCloudPtr ,filtered_cloud);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 depth_combined;
  pcl_conversions::moveFromPCL(filtered_cloud, depth_combined);

  // Publish the data
  pub.publish (depth_combined);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "combine");
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
  pub = nh.advertise<sensor_msgs::PointCloud2>("depth_combined", 1);

  //tf2 listener
  tf2_listener = new tf2_ros::TransformListener(tfBuffer);
  
  //Loop rate 10 Hz
  ros::Rate loop_rate(10);

  // Spin
  ros::spin ();
}