/** * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 7/11/19
 *     Author: Francesco Verdoja <francesco.verdoja@aalto.fi>
 */

#ifndef SUPER_CLUSTERING_SEGMENTATION_H
#define SUPER_CLUSTERING_SEGMENTATION_H

#include <point_cloud_segmentation/SuperClusteringSegParams.h>
#include <point_cloud_segmentation/base_segmentation.h>
class SuperClusteringSegmentation : public BaseSegmentation {
public:
  SuperClusteringSegmentation();
  virtual void segment_scene(PointCloudPtr, std::vector<pcl::PointIndices> &,
                             std::vector<PointCloud> &);
  bool update_parameters_cb(
      point_cloud_segmentation::SuperClusteringSegParams::Request &,
      point_cloud_segmentation::SuperClusteringSegParams::Response &);

private:
  float voxel_resolution;
  float seed_resolution;
  float color_importance;
  float spatial_importance;
  float normal_importance;
  float threshold;
  bool rgb_color_space;
  bool convexity_criterion;
  bool adapt_lambda;
  bool equalization;

  ros::ServiceServer update_params_srv;
};

#endif // SUPER_CLUSTERING_SEGMENTATION_H
