#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>

#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>

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
  pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloudFilteredPtr(xyz_cloud_filtered);

  // Convert the pcl::PointCloud2 type to pcl::PointCloud<pcl::PointXYZ>
  pcl::fromPCLPointCloud2(*cloud, *xyzCloudPtr);

  // Build a passthrough filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-100, 100);
  pass.filter (*xyzCloudFilteredPtr); 

//   ## RANSAC plane method ############################################################################################
// ## Comment : Works well with distance threshold = 0.11###############################################################

  // Create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZ> *xyz_ground_ransac = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzGroundPtrRansac(xyz_ground_ransac);
  pcl::PointCloud<pcl::PointXYZ> *xyz_obs_ransac = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzObsPtrRansac(xyz_obs_ransac);

  // RANSAC filtration
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr ground (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.11);
  seg.setInputCloud (xyzCloudFilteredPtr);
  seg.segment (*ground, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract Ground
  extract.setInputCloud (xyzCloudFilteredPtr);
  extract.setIndices (ground);
  extract.filter (*xyzGroundPtrRansac);

  // Extract Obstacle
  extract.setInputCloud (xyzCloudFilteredPtr);
  extract.setIndices (ground);
  extract.setNegative (true);
  extract.filter (*xyzObsPtrRansac);

  // Declare the output variables
  pcl::PCLPointCloud2 ground_output_pcl;
  pcl::PCLPointCloud2 obs_output_pcl;

  // Convert to pcl::PCLPointCloud2
  pcl::toPCLPointCloud2( *xyzGroundPtrRansac ,ground_output_pcl);
  pcl::toPCLPointCloud2( *xyzObsPtrRansac ,obs_output_pcl);

// ##########################################################################################################

//   ## RANSAC normal plane method ############################################################################################
// ## Comment : Require more test to get the optimal param#####################################################################

//   // Create a pcl object to hold the ransac filtered results
//   pcl::PointCloud<pcl::PointXYZ> *xyz_ground_ransac = new pcl::PointCloud<pcl::PointXYZ>;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr xyzGroundPtrRansac(xyz_ground_ransac);
//   pcl::PointCloud<pcl::PointXYZ> *xyz_obs_ransac = new pcl::PointCloud<pcl::PointXYZ>;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr xyzObsPtrRansac(xyz_obs_ransac);

//   // Estimate Point Normal
//   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>); 
//   ne.setSearchMethod (tree);
//   ne.setInputCloud (xyzCloudFilteredPtr);
//   ne.setKSearch (50);
//   ne.compute (*cloud_normals); 

//   //RANSAC filtration
//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr ground (new pcl::PointIndices);

//   // Create the segmentation object for the planar model and set all the parameters
//   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
//   seg.setOptimizeCoefficients (true);
//   seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//   seg.setNormalDistanceWeight (0.005);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (100);
//   seg.setDistanceThreshold (0.03);
//   seg.setInputCloud (xyzCloudFilteredPtr);
//   seg.setInputNormals (cloud_normals);
  
//   // Obtain the plane inliers and coefficients
//   pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
//   seg.segment (*inliers_plane, *coefficients_plane); 

//   // Create the filtering object
//   pcl::ExtractIndices<pcl::PointXYZ> extract;

//   // Extract Ground
//   extract.setInputCloud (xyzCloudFilteredPtr);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (false);
//   extract.filter (*xyzGroundPtrRansac);

//   // Extract Obstacle
//   extract.setInputCloud (xyzCloudFilteredPtr);
//   extract.setIndices (inliers_plane);
//   extract.setNegative (true);
//   extract.filter (*xyzObsPtrRansac);

//   // Declare the output variables
//   pcl::PCLPointCloud2 ground_output_pcl;
//   pcl::PCLPointCloud2 obs_output_pcl;

//   // Convert to pcl::PCLPointCloud2
//   pcl::toPCLPointCloud2( *xyzGroundPtrRansac ,ground_output_pcl);
//   pcl::toPCLPointCloud2( *xyzObsPtrRansac ,obs_output_pcl);

// ##########################################################################################################
  
 // ## Progressive Morphological Filter method ##############################################################
 // ## Comment : Too slow for online process ################################################################
 
//   // Create a pcl object to hold the filtered results
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointIndicesPtr ground (new pcl::PointIndices);

//   // Create the filtering object
//   pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
//   pmf.setInputCloud (xyzCloudFilteredPtr);
//   pmf.setMaxWindowSize (20);
//   pmf.setSlope (1.0f);
//   pmf.setInitialDistance (0.13f);
//   pmf.setMaxDistance (3.0f);
//   pmf.extract (ground->indices);

//   // Create a pcl object to hold the filtered results
//   pcl::PointCloud<pcl::PointXYZ> *xyz_ground_pmf = new pcl::PointCloud<pcl::PointXYZ>;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr xyzGroundPtrPMF(xyz_ground_pmf);
//   pcl::PointCloud<pcl::PointXYZ> *xyz_obs_pmf = new pcl::PointCloud<pcl::PointXYZ>;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr xyzObsPtrPMF(xyz_obs_pmf);
  
//   // Extract Ground
//   pcl::ExtractIndices<pcl::PointXYZ> extract;
//   extract.setInputCloud(xyzCloudFilteredPtr);
//   extract.setIndices(ground);
//   extract.filter(*xyzGroundPtrPMF);

//   // Extract Obstacle
//   extract.setInputCloud(xyzCloudFilteredPtr);
//   extract.setIndices(ground);
//   extract.setNegative (true);
//   extract.filter (*xyzObsPtrPMF);

//   // Declare the PCL output variables
//   pcl::PCLPointCloud2 ground_output_pcl;
//   pcl::PCLPointCloud2 obs_output_pcl;

//   // Convert to pcl::PCLPointCloud2
//   pcl::toPCLPointCloud2( *xyzGroundPtrPMF ,ground_output_pcl);
//   pcl::toPCLPointCloud2( *xyzObsPtrPMF ,obs_output_pcl);

  // ##########################################################################################################
  
  // Declare the output variables
  sensor_msgs::PointCloud2 ground_out;
  sensor_msgs::PointCloud2 obs_out;
  
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