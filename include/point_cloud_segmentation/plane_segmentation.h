/** * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 7/4/19
 *     Author: Jens Lundell <jens.lundell@aalto.fi>
 */

#ifndef PLANE_SEGMENTATION_H
#define PLANE_SEGMENTATION_H

#include <point_cloud_segmentation/PlaneSegmentationParams.h>
#include <point_cloud_segmentation/base_segmentation.h>
class PlaneSegmentation : public BaseSegmentation {
public:
  PlaneSegmentation();
  bool update_parameters_cb(
      point_cloud_segmentation::PlaneSegmentationParams::Request &,
      point_cloud_segmentation::PlaneSegmentationParams::Response &);
  virtual void segment_scene(PointCloudPtr, std::vector<pcl::PointIndices> &,
                             std::vector<PointCloud> &);

private:
  float inlier_threshold;
  int max_iters;
  ros::ServiceServer update_params_srv;
};

#endif // PLANE_SEGMENTATION_H
