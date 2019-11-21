/** * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 7/4/19
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#ifndef BASE_SEGMENTATION_H
#define BASE_SEGMENTATION_H

#include <cstdlib>
#include <mutex>
#include <string>
#include <time.h>
// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
// ROS
#include <point_cloud_segmentation/BaseSegmentationParams.h>
#include <point_cloud_segmentation/ChangeSegmentationMethod.h>
#include <point_cloud_segmentation/SegmentScene.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_ros/point_cloud.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
// VTK
#include <vtkPolyLine.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

class BaseSegmentation
{
public:
  BaseSegmentation();
  virtual void segment_scene(PointCloudPtr, std::vector<pcl::PointIndices> &,
                             std::vector<PointCloud> &);
  void segment_scene_callback(const PointCloudConstPtr &);
  bool segment_scene_service(point_cloud_segmentation::SegmentScene::Request &,
                             point_cloud_segmentation::SegmentScene::Response &);
  void downsample_PC(PointCloudPtr, const float leaf_size);
  uint32_t generate_random_color();
  void pcl_indices_to_ros_msg(const std::vector<pcl::PointIndices> &,
                              std::vector<pcl_msgs::PointIndices> &);
  void remove_points_not_within_bounds(PointCloudPtr pc);
  void pcl_pcs_to_ros_pcs(const std::vector<PointCloud> &,
                          std::vector<sensor_msgs::PointCloud2> &);
  ;

  bool within_bounds(PointT &p);
  bool
  update_parameters_cb(point_cloud_segmentation::BaseSegmentationParams::Request &,
                       point_cloud_segmentation::BaseSegmentationParams::Response &);
  void preprocess_pc(PointCloud, PointCloudPtr);

protected:
  ros::NodeHandle node;
  ros::ServiceServer segment_scene_srv;
  ros::Publisher pub_pc;
  ros::Subscriber sub_pc;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  std::string camera_frame;
  std::string base_frame;
  std::vector<float> z_cutoff;
  std::vector<float> x_cutoff;
  std::vector<float> y_cutoff;

private:
  std::mutex mutex_data;
  ros::ServiceServer update_params_srv;
};

#endif // BASE_SEGMENTATION_H
