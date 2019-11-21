#include "point_cloud_segmentation/super_clustering_segmentation.h"
#include "supervoxel_clustering/clustering.h"

SuperClusteringSegmentation::SuperClusteringSegmentation()
    : BaseSegmentation() {
  voxel_resolution = node.param<float>("voxel_resolution", 0.008);
  seed_resolution = node.param<float>("seed_resolution", 0.1);
  color_importance = node.param<float>("color_importance", 0.2);
  spatial_importance = node.param<float>("spatial_importance", 0.4);
  normal_importance = node.param<float>("normal_importance", 1);
  threshold = node.param<float>("threshold", 0.2);
  rgb_color_space = node.param<bool>("rgb_color_space", false);
  convexity_criterion = node.param<bool>("convexity_criterion", false);
  adapt_lambda = node.param<bool>("adapt_lambda", false);
  equalization = node.param<bool>("equalization", false);
  update_params_srv = node.advertiseService(
      "point_cloud_segmentation/update_params",
      &SuperClusteringSegmentation::update_parameters_cb, this);
}

bool SuperClusteringSegmentation::update_parameters_cb(
    point_cloud_segmentation::SuperClusteringSegParams::Request &req,
    point_cloud_segmentation::SuperClusteringSegParams::Response &res) {
  voxel_resolution = req.voxel_resolution;
  seed_resolution = req.seed_resolution;
  color_importance = req.color_importance;
  spatial_importance = req.spatial_importance;
  normal_importance = req.normal_importance;
  threshold = req.threshold;
  rgb_color_space = req.rgb_color_space;
  convexity_criterion = req.convexity_criterion;
  adapt_lambda = req.adapt_lambda;
  equalization = req.equalization;
  return true;
}

void SuperClusteringSegmentation::segment_scene(
    PointCloudPtr pc, std::vector<pcl::PointIndices> &cluster_indices,
    std::vector<PointCloud> &segmented_pcs) {

  if (pc->points.size() == 0)
    return;

  if (!(adapt_lambda || equalization)) {
    adapt_lambda = true;
    ROS_DEBUG(
        "No merging method specified, Adaptive Lambda is going to be used\n");
  } else if (adapt_lambda && equalization) {
    ROS_ERROR("Only one merging method can be specified at a time, choose "
              "either adapt_lambda or equalization\n");
    return;
  }

  ////////////////////////////////////////////////////////////
  ////// Supervoxel generation
  ////////////////////////////////////////////////////////////

  pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
  super.setInputCloud(pc);
  super.setColorImportance(color_importance);
  super.setSpatialImportance(spatial_importance);
  super.setNormalImportance(normal_importance);

  ClusteringT supervoxel_clusters;

  // ROS_INFO("Extracting supervoxels...\n");

  super.extract(supervoxel_clusters);

  // ROS_INFO("Found %d supervoxels\n", supervoxel_clusters.size());

  AdjacencyMapT supervoxel_adjacency;
  super.getSupervoxelAdjacency(supervoxel_adjacency);
  // To make a graph of the supervoxel adjacency, we need to iterate through the
  // supervoxel adjacency multimap

  ////////////////////////////////////////////////////////////
  ////// Segmentation
  ////////////////////////////////////////////////////////////

  // ROS_INFO("Segmentation initialization...\n");

  Clustering segmentation;
  if (rgb_color_space)
    segmentation.set_delta_c(RGB_EUCL);

  if (convexity_criterion)
    segmentation.set_delta_g(CONVEX_NORMALS_DIFF);

  if (equalization)
    segmentation.set_merging(EQUALIZATION);

  segmentation.set_initialstate(supervoxel_clusters, supervoxel_adjacency);
  if (adapt_lambda)
    ROS_DEBUG("Lambda: %f\n", segmentation.get_lambda());

  // ROS_INFO("Initialization complete\nStarting clustering...\n");

  segmentation.cluster(threshold);

  // ROS_INFO("Clustering complete\n");

  *pc = *(segmentation.get_colored_cloud());
  ClusteringT state = segmentation.get_currentstate().first;
  for (auto it = state.begin(); it != state.end(); it++) {
    segmented_pcs.push_back(*(it->second->voxels_));
  }
}
