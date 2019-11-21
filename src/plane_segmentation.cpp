#include "point_cloud_segmentation/plane_segmentation.h"

PlaneSegmentation::PlaneSegmentation() : BaseSegmentation() {
  inlier_threshold = node.param<float>("inlier_threshold", 0.001);
  max_iters = node.param<int>("max_iterations", 100);
  update_params_srv =
      node.advertiseService("point_cloud_segmentation/update_params",
                            &PlaneSegmentation::update_parameters_cb, this);
}

bool PlaneSegmentation::update_parameters_cb(
    point_cloud_segmentation::PlaneSegmentationParams::Request &req,
    point_cloud_segmentation::PlaneSegmentationParams::Response &res) {
  inlier_threshold = req.inlier_threshold;
  max_iters = req.max_iters;
  return true;
}

void PlaneSegmentation::segment_scene(
    PointCloudPtr pc, std::vector<pcl::PointIndices> &cluster_indices,
    std::vector<PointCloud> &segmented_pcs) {

  PointCloudPtr cloud_f(new PointCloud);
  pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iters);
  seg.setDistanceThreshold(inlier_threshold);
  int i = 0, nr_points = (int)pc->points.size();
  while (pc->points.size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(pc);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(pc);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *pc = *cloud_f;
  }

  return;
}
