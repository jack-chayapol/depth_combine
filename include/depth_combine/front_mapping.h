#ifndef FRONT_MAPPING_H
#define FRONT_MAPPING_H

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>

class FrontMapping
{
    private:
      ros::NodeHandle nh;
      ros::Subscriber msg_sub;
      ros::Publisher msg_pub;
    
    public:
      FrontMapping();
      template <typename T>
      void FrontCB(const sensor_msgs::PointCloud2::ConstPtr& forward);
};

#endif