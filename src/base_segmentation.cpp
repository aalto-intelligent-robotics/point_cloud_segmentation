#include "point_cloud_segmentation/base_segmentation.h"

BaseSegmentation::BaseSegmentation() : node("~"), tf_listener(tf_buffer) {
  std::string pc_topic =
      node.param<std::string>("pc_topic", "/camera/depth_registered/points");
  camera_frame = node.param<std::string>("camera_frame", "camera_rgb_optical_frame");
  base_frame = node.param<std::string>("base_frame", "camera_rgb_optical_frame");
  x_cutoff = node.param<std::vector<float>>("x_cutoff",
                                            std::vector<float>{-1000, 1000});
  y_cutoff = node.param<std::vector<float>>("y_cutoff",
                                            std::vector<float>{-1000, 1000});
  z_cutoff = node.param<std::vector<float>>("z_cutoff",
                                            std::vector<float>{0.001, 1000});
  segment_scene_srv =
      node.advertiseService("point_cloud_segmentation/scene_segmentation",
                            &BaseSegmentation::segment_scene_service, this);
  update_params_srv =
      node.advertiseService("point_cloud_segmentation/update_base_params",
                            &BaseSegmentation::update_parameters_cb, this);
  pub_pc =
      node.advertise<PointCloud>("point_cloud_segmentation/segmented_pc", 1000);
  sub_pc = node.subscribe<PointCloud>(
      pc_topic, 1, &BaseSegmentation::segment_scene_callback, this);
}

bool BaseSegmentation::update_parameters_cb(
    point_cloud_segmentation::BaseSegmentationParams::Request &req,
    point_cloud_segmentation::BaseSegmentationParams::Response &res) {
  x_cutoff = req.x_cutoff;
  y_cutoff = req.y_cutoff;
  z_cutoff = req.z_cutoff;
  return true;
}

void BaseSegmentation::segment_scene(
    PointCloudPtr pc, std::vector<pcl::PointIndices> &cluster_indices,
    std::vector<PointCloud> &segmented_pcs) {
  segmented_pcs.push_back(*pc.get());
  return;
}

bool BaseSegmentation::segment_scene_service(
    point_cloud_segmentation::SegmentScene::Request &req,
    point_cloud_segmentation::SegmentScene::Response &res) {
  PointCloudPtr received_cloud_ptr;
  received_cloud_ptr.reset(new PointCloud);
  PointCloud input_pc;
  pcl::fromROSMsg(req.cloud_in, input_pc);
  preprocess_pc(input_pc, received_cloud_ptr);
  std::vector<pcl::PointIndices> indices;
  std::vector<PointCloud> pcs;
  segment_scene(received_cloud_ptr, indices, pcs);
  pcl::toROSMsg(*received_cloud_ptr.get(), res.segmented_scene.colored_cloud);
  pcl_indices_to_ros_msg(indices, res.segmented_scene.Indices);
  pcl_pcs_to_ros_pcs(pcs, res.segmented_scene.segmented_clouds);
  return true;
}

void BaseSegmentation::segment_scene_callback(const PointCloudConstPtr &input) {
  if (pub_pc.getNumSubscribers() != 0) {
    PointCloudPtr received_cloud_ptr;
    received_cloud_ptr.reset(new PointCloud);
    preprocess_pc(*input.get(), received_cloud_ptr);
    std::vector<pcl::PointIndices> indices;
    std::vector<PointCloud> pcs;
    segment_scene(received_cloud_ptr, indices, pcs);
    point_cloud_segmentation::SegmentedPointCloud segmented_scene;
    sensor_msgs::PointCloud2 segmented_pc;
    pcl::toROSMsg(*received_cloud_ptr.get(), segmented_pc);

    segmented_pc.header.frame_id = camera_frame;
    pub_pc.publish(segmented_pc);

    ros::Duration(1).sleep();
  }
}

void BaseSegmentation::pcl_indices_to_ros_msg(
    const std::vector<pcl::PointIndices> &pcl_ind,
    std::vector<pcl_msgs::PointIndices> &pcl_ind_msg) {
  pcl_msgs::PointIndices pi;
  for (auto const &iter : pcl_ind) {
    pcl_conversions::fromPCL(iter, pi);
    pcl_ind_msg.push_back(pi);
  }
}

void BaseSegmentation::pcl_pcs_to_ros_pcs(
    const std::vector<PointCloud> &pcl_pcs,
    std::vector<sensor_msgs::PointCloud2> &ros_pcs) {
  std_msgs::Header header;
  header.frame_id = "camera_link";
  header.stamp = ros::Time::now();
  sensor_msgs::PointCloud2 pc;
  for (auto const &iter : pcl_pcs) {
    pcl::toROSMsg(iter, pc);
    pc.header = header;
    ros_pcs.push_back(pc);
  }
}

void BaseSegmentation::downsample_PC(PointCloudPtr cloud,
                                     const float leaf_size) {
  PointCloudPtr cloud_filtered(new PointCloud);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(*cloud_filtered);
  *cloud = *cloud_filtered;
}

uint32_t BaseSegmentation::generate_random_color() {
  uint8_t r = rand() % 256, g = rand() % 256, b = rand() % 256;
  return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
}

void BaseSegmentation::remove_points_not_within_bounds(PointCloudPtr pc) {

  geometry_msgs::TransformStamped cam_to_base;
  try {
    cam_to_base = tf_buffer.lookupTransform(
        base_frame, camera_frame, ros::Time::now(), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_ERROR_STREAM(
        "Cannot find transformation from camera to base. What: " << ex.what());
    return;
  }

  Eigen::Affine3d transform = tf2::transformToEigen(cam_to_base);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::transformPointCloud(*pc, *transformed_cloud, transform);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;

#pragma omp parallel for
  for (int i = 0; i < (*pc).size(); i++) {
    if (within_bounds(transformed_cloud->points[i])) {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(pc);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*pc);
}

bool BaseSegmentation::within_bounds(PointT &p) {
  if (p.z > z_cutoff[0] and p.z < z_cutoff[1])
    if (p.x > x_cutoff[0] and p.x < x_cutoff[1])
      if (p.y > y_cutoff[0] and p.y < y_cutoff[1])
        return true;
  return false;
}

void BaseSegmentation::preprocess_pc(PointCloud input_pc,
                                     PointCloudPtr processed_pc) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(input_pc, input_pc, indices);
  *processed_pc = input_pc;
  remove_points_not_within_bounds(processed_pc);
}
