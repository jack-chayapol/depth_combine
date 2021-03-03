#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

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


using namespace message_filters;

ros::Publisher pub;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tf2_listener;

void 
obs_cb (const sensor_msgs::PointCloud2ConstPtr& obs , const sensor_msgs::PointCloud2ConstPtr& velodyne)
{
 
  //Container for converted cloud
  sensor_msgs::PointCloud2 conv_cloud_obs;
  sensor_msgs::PointCloud2 conv_cloud_velodyne;

  //Transform Pointcloud
  try 
    {
        pcl_ros::transformPointCloud ("base_link", *obs, conv_cloud_obs, tfBuffer);
        pcl_ros::transformPointCloud ("base_link", *velodyne, conv_cloud_velodyne, tfBuffer);
              
      ROS_INFO("tf received\n");
    }
  catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud_obs = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2* cloud_velodyne = new pcl::PCLPointCloud2; 
  
  // Convert to PCL data type
  pcl_conversions::toPCL(conv_cloud_obs, *cloud_obs);
  pcl_conversions::toPCL(conv_cloud_velodyne, *cloud_velodyne);

  // Convert the velodyne cloud from pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZ> to remove unwanted field
  pcl::PointCloud<pcl::PointXYZ> *velodyne_xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::fromPCLPointCloud2(*cloud_velodyne, *velodyne_xyz_cloud);
  pcl::PointCloud<pcl::PointXYZ> *velodyne_temp = new pcl::PointCloud<pcl::PointXYZ>;

  *velodyne_temp = *velodyne_xyz_cloud;

  // Convert back to pcl::PCLPointCloud2
  pcl::PCLPointCloud2* velodyne_converted = new pcl::PCLPointCloud2;
  pcl::toPCLPointCloud2( *velodyne_temp ,*velodyne_converted);

  // Combine clouds
  pcl::PCLPointCloud2* combined_obstacle = new pcl::PCLPointCloud2; 
  pcl::concatenatePointCloud (*combined_obstacle, *cloud_obs, *combined_obstacle);
  pcl::concatenatePointCloud (*combined_obstacle, *velodyne_converted, *combined_obstacle);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 obs_combined;
  pcl_conversions::moveFromPCL(*combined_obstacle, obs_combined);

  // Publish the data
  pub.publish (obs_combined);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obs_combine");
  ros::NodeHandle nh;

  //Synchronize the message before calling obs_cb
  message_filters::Subscriber<sensor_msgs::PointCloud2> obs_sub(nh, "/obs_cloud", 1000);
  // message_filters::Subscriber<sensor_msgs::PointCloud2> obs_sub(nh, "/depth_combined", 1000);
  message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh, "/upper_velodyne_vlp16/depth/points", 1000);
  
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), obs_sub, velodyne_sub);
  sync.registerCallback(boost::bind(&obs_cb, _1, _2));

  
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("obstacle", 1);

  //tf2 listener
  tf2_listener = new tf2_ros::TransformListener(tfBuffer);
  
  //Loop rate 10 Hz
  ros::Rate loop_rate(10);

  // Spin
  ros::spin ();
}