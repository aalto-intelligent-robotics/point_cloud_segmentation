#include "point_cloud_segmentation/region_growing_segmentation.h"

RegionGrowingSegmentation::RegionGrowingSegmentation() : BaseSegmentation() {
  min_cluster_size = node.param<int>("min_cluster_size", 300);
  max_cluster_size = node.param<int>("max_cluster_size", 140000);
  number_of_neighbors = node.param<int>("number_of_neighbors", 30);
  smoothness_threshold =
      node.param<float>("smoothness_threshold", 3.0 / 180.0 * M_PI);
  curvature_threshold = node.param<float>("curvature_threshold", 1);
  update_params_srv = node.advertiseService(
      "point_cloud_segmentation/update_params",
      &RegionGrowingSegmentation::update_parameters_cb, this);
}

bool RegionGrowingSegmentation::update_parameters_cb(
    point_cloud_segmentation::RegionGrowingSegParams::Request &req,
    point_cloud_segmentation::RegionGrowingSegParams::Response &res) {
  min_cluster_size = req.min_cluster_size;
  max_cluster_size = req.max_cluster_size;
  number_of_neighbors = req.number_of_neighbors;
  smoothness_threshold = req.smoothness_threshold;
  curvature_threshold = req.curvature_threshold;
  return true;
}

void RegionGrowingSegmentation::segment_scene(
    PointCloudPtr pc, std::vector<pcl::PointIndices> &cluster_indices,
    std::vector<PointCloud> &segmented_pcs) {

  pcl::search::Search<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(pc);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  // pcl::IndicesPtr indices (new std::vector <int>);
  // pcl::PassThrough<PointT> pass;
  // pass.setInputCloud (pc);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // pass.filter (*indices);

  pcl::RegionGrowing<PointT, pcl::Normal> reg;
  reg.setMinClusterSize(min_cluster_size);
  reg.setMaxClusterSize(max_cluster_size);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(number_of_neighbors);
  reg.setInputCloud(pc);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smoothness_threshold);
  reg.setCurvatureThreshold(curvature_threshold);

  reg.extract(cluster_indices);

  // std::cout << "Number of clusters is equal to " << cluster_indices.size ()
  // << std::endl;
  PointCloudPtr cloud_f(new PointCloud);
  *cloud_f = *pc;

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
