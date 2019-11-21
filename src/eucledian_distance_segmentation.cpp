#include "point_cloud_segmentation/eucledian_cluster_extraction.h"

EucledianClusterExtraction::EucledianClusterExtraction() : BaseSegmentation() {
  inlier_threshold = node.param<float>("inlier_threshold", 0.001);
  max_iters = node.param<int>("max_iterations", 100);
  leaf_size = node.param<float>("leaf_size", 0.1);
  min_cluster_size = node.param<int>("min_cluster_size", 100);
  max_cluster_size = node.param<int>("max_cluster_size", 140000);
  cluster_tolerance = node.param<float>("cluster_tolerance", 0.02);
  update_params_srv = node.advertiseService(
      "point_cloud_segmentation/update_params",
      &EucledianClusterExtraction::update_parameters_cb, this);
}

bool EucledianClusterExtraction::update_parameters_cb(
    point_cloud_segmentation::EucledianClusterParams::Request &req,
    point_cloud_segmentation::EucledianClusterParams::Response &res) {
  inlier_threshold =
      req.inlier_threshold > 0 ? req.inlier_threshold : inlier_threshold;
  max_iters = req.max_iters > 0 ? req.max_iters : max_iters;
  leaf_size = req.leaf_size > 0 ? req.leaf_size : leaf_size;
  min_cluster_size =
      req.min_cluster_size > 0 ? req.min_cluster_size : min_cluster_size;
  max_cluster_size =
      req.max_cluster_size > 0 ? req.max_cluster_size : max_cluster_size;
  cluster_tolerance =
      req.cluster_tolerance > 0 ? req.cluster_tolerance : cluster_tolerance;
  return true;
}

void EucledianClusterExtraction::segment_scene(
    PointCloudPtr pc, std::vector<pcl::PointIndices> &cluster_indices,
    std::vector<PointCloud> &segmented_pcs) {

  if (pc->points.size() == 0) {
    ROS_WARN("Pointcloud buffer is empty!");
    return;
  }
  PointCloudPtr cloud_f(new PointCloud);
  *cloud_f = *pc;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_f);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_f);
  ec.extract(cluster_indices);

  // std::cout<<"Number of clusters: "<<cluster_indices.size()<<std::endl;
  uint32_t rgb;
  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    rgb = generate_random_color();
    PointCloudPtr cloud_cluster(new PointCloud);
#pragma omp parallel for
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      cloud_cluster->points.push_back(cloud_f->points[*pit]);
      cloud_f->points[*pit].rgb = rgb;
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    segmented_pcs.push_back(*cloud_cluster);
  }
  *pc = *cloud_f;
  return;
}
