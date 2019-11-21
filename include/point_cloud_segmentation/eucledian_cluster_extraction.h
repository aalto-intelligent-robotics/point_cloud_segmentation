/** * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 7/4/19
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#ifndef EUCLEDIAN_CLUSTER_EXTRACTION_H
#define EUCLEDIAN_CLUSTER_EXTRACTION_H

#include <point_cloud_segmentation/EucledianClusterParams.h>
#include <point_cloud_segmentation/base_segmentation.h>
class EucledianClusterExtraction : public BaseSegmentation {
public:
  EucledianClusterExtraction();
  virtual void segment_scene(PointCloudPtr, std::vector<pcl::PointIndices> &,
                             std::vector<PointCloud> &);
  bool
  update_parameters_cb(point_cloud_segmentation::EucledianClusterParams::Request &,
                       point_cloud_segmentation::EucledianClusterParams::Response &);

private:
  float inlier_threshold;
  int max_iters;
  float leaf_size;
  int min_cluster_size;
  int max_cluster_size;
  float cluster_tolerance;
  ros::ServiceServer update_params_srv;
};

#endif // PLANE_SEGMENTATION_H
