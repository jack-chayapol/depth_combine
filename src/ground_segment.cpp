#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

// Segmentation class
class segmentation  {

    public:

        explicit segmentation(ros::NodeHandle nh) : seg_nh(nh) {
            seg_sub = seg_nh.subscribe ("/depth_combined", 1, &segmentation::cloud_cb, this);
            seg_pub_ground = seg_nh.advertise<sensor_msgs::PointCloud2> ("ground_cloud", 1);
            seg_pub_obs = seg_nh.advertise<sensor_msgs::PointCloud2> ("obs_cloud", 1);
        }
    
    private:

    ros::NodeHandle seg_nh;
    ros::Publisher seg_pub_ground;
    ros::Publisher seg_pub_obs;
    ros::Subscriber seg_sub;

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_in);

};

void segmentation::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

  // Convert to PCL data type
  pcl_conversions::toPCL( *cloud_in, *cloud );

  // Container for pcl objects
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudPtr(xyz_cloud);

  // Convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(*cloud, *xyzCloudPtr);

  // Create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZ> *xyz_ground_ransac = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzGroundPtrRansac(xyz_ground_ransac);
  pcl::PointCloud<pcl::PointXYZ> *xyz_obs_ransac = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzObsPtrRansac(xyz_obs_ransac);

  //RANSAC filtration
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.06);
  seg.setInputCloud (xyzCloudPtr);
  seg.segment (*ground, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract Ground
  extract.setInputCloud (xyzCloudPtr);
  extract.setIndices (ground);
  extract.filter (*xyzGroundPtrRansac);

  // Extract Obstacle
  extract.setInputCloud (xyzCloudPtr);
  extract.setIndices (ground);
  extract.setNegative (true);
  extract.filter (*xyzObsPtrRansac);

  // declare the output variables
  sensor_msgs::PointCloud2 ground_out;
  sensor_msgs::PointCloud2 obs_out;
  pcl::PCLPointCloud2 ground_output_pcl;
  pcl::PCLPointCloud2 obs_output_pcl;

  // Convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzGroundPtrRansac ,ground_output_pcl);
  pcl::toPCLPointCloud2( *xyzObsPtrRansac ,obs_output_pcl);

  // Convert to ROS data type
  pcl_conversions::moveFromPCL(ground_output_pcl, ground_out);
  pcl_conversions::moveFromPCL(obs_output_pcl, obs_out);

  // Publish the data
  seg_pub_ground.publish (ground_out);
  seg_pub_obs.publish (obs_out);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ground_segment");
  ros::NodeHandle nh;
  
  segmentation segs(nh);

  ros::Rate loop_rate(10);

  // Spin
  while(ros::ok()){
  ros::spin ();
  loop_rate.sleep();
  }
}