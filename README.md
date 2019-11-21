# Point-Cloud Segmentation

This repository includes classical segmentation methods for Point-Clouds (PCs) implemented as a Robotic-Operating-System (ROS) node. The implemented segmentation methods are:

- [Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction),
- [Region Growing Segmentation](http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation),
- [Plane model Segmentation](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation),
- [Supervoxel Segmentation](https://github.com/fverdoja/Fast-3D-Pointcloud-Segmentation).


**Authors**: Jens Lundell and Francesco Verdoja  
**Maintainer**: Jens Lundell, jens.lundell@aalto.fi  
**Affiliation**: Intelligent Robotics Lab, Aalto University

## Getting Started

### Dependencies

[ROS Melodic](http://wiki.ros.org/melodic)

[Fast 3D Pointcloud Segmentation](https://github.com/fverdoja/Fast-3D-Pointcloud-Segmentation)

### ROS Installation

Clone or download the project from Github:

```
cd <PATH_TO_YOUR_CATKIN_WORKSPACE>/src
git clone git@github.com:aalto-intelligent-robotics/point_cloud_segmentation.git
```

Compile the ROS workspace

```
cd <PATH_TO_YOUR_CATKIN_WORKSPACE>
catkin_make
```
## Running the Segmentation

The simplest option to run the segmentation node is to do the following

```
roslaunch point_cloud_segmentation segmentation.launch
```

## Testing the Segmentation with a Kinect 360 and rviz

First start the Kinect camera

```
roslaunch openni_launch openni.launch
```

Then launch the segmentation node

```
roslaunch point_cloud_segmentation segmentation.launch segmentation_algorithm="region"
```

Finally run rviz
```
roscd point_cloud_segmentation
rosrun rviz rviz -d rviz/point_cloud_segmentation_config.rviz
```
### Before segmentation 

![Original Scene](images/original_scene.png?raw=true "Original Scene")

### After segmentation 

![Segmented Scene](images/segmented_scene.png?raw=true "Segmented Scene")


# Node: point_cloud_segmentation 

`point_cloud_segmentation` takes a point-cloud as input and generates a segmented point-cloud. The node uses lazy publishing and will not publish the segmented point-cloud until there is a subscriber for `point_cloud_segmentation/segmented_pc`.

## Parameters

`~pc_topic` (sensor_msgs/PointCloud, default: /camera/depth_registered/points) - The input point-cloud topic.

`~camera_frame` (String, default camera_frame) - Name of the camera frame used.

`~base_frame` (String. default base_link) - Name of the base frame that the point-cloud is transformed into for doing background subtraction. One optioni for setting the base_frame is to estimate the plane where the objects are lying with an Aruco marker.

`~x_cutoff` ([float, float], default [-0.3, 0.3]) - Points outside this region in x-direction are removed.

`~y_cutoff` ([float, float], default [-0.3, 0.3]) - Points outside this region in y-direction are removed.

`~z_cutoff` ([float, float], default [0.001, 0.3]) - Points outside this region in z-direction are removed.

`~segmentation_algorithm` (String, default base) - The segmentation method to use. Base method only does background subtraction. The other alternatives are: plane, eucledian, region, superclustering.

`~inlier_threshold` (float, default 0.01) - Determines how close a point must be to the model in order to be considered an inlier.

`~leaf_size` (float, default 0.005) - Specifies the down-sampling rate of the point-cloud when using the Eucledian Cluster Segmentation method.

`~min_cluster_size` (int, default 300) - The minimum number of points a cluster need to have to be considered a segment. Only used for Eucledian Cluster Segmentation and Region Growing Segmentation.

`~max_cluster_size` (int, default 100000) - The maximum number of points a cluster can have to be considered a segment.

`~cluster_tolerance` (float, default 0.01) - the maximum distance a point may have to a cluster member to also be counted as member of that cluster. Only used for Eucledian cluster segmentation.

`~curvature_threshold` (float, default 1) - The maximum deviation between the normals of two points to be in the same cluster. 

`~smoothness_threshold` (float, default 0.4) - The maximum deviation between point normals to be in the same cluster. Only used for Region Growing Segmentation.

`~number_of_neighbours` (int, default 80) - number of neighbors to use for the K nearest search. Only used for Region Growing Segmentation.

`~voxel_resolution` (float, default 0.005) - Sets the voxel size, which determines the leaf size of the underlying octree structure (in meters). Only used for Supervoxel Segmentation.

`~seed_resolution` (float, default 0.1) - Sets the seeding size, which determines how big the supervoxels will be (in meters). Only used for Supervoxel Segmentation.

`~color_importance` (float, default 0.2) - Sets the weight for color - how much color will influence the shape of the supervoxels. Only used for Supervoxel Segmentation.

`~spatial_importance` (float, default 0.4) - Sets the weight for color - how much color will influence the shape of the supervoxels. Only used for Supervoxel Segmentation.

`~normal_importance` (float, default 1.0) - Sets the weight for normal - how much surface normals will influence the shape of the supervoxels. Only used for Supervoxel Segmentation.

`~threshold`. (float, 0.5) - Determines which level of the dendogram represents the best segmentation. Only used for Supervoxel Segmentation.

`~rgb_color_space` (bool, default false) - Uses the RGB color space for measuring the color distance; if false, L*A*B* color space is used. Only used for Supervoxel Segmentation.

`~convexity_criterion` (bool, default true) - Uses the convexity criterion to weigh the geometric distance; if false, convexity is not considered. Only used for Supervoxel Segmentation.

`~adapt_lambda` (bool, default false) - Uses Adaptive lambda as merging criterion. Only used for Supervoxel Segmentation.

`~equalization` (bool, default false) - Uses Equalization as merging criterion; if false, 200 bins are used. Only used for Supervoxel Segmentation.

## Subscribed Topics

`pc_topic` (sensor_msgs/PointCloud) - The topic for the input point-cloud. This is typically set to /camera/depth_registered/points if you use a Kinect camera.

## Published Topics

`point_cloud_segmentation/segmented_pc` (sensor_msgs/PointCloud) - The output segmented point-cloud color coded according to the segments.

## Services

`point_cloud_segmentation/scene_segmentation` (input: sensor_msgs/PointCloud, output: point_cloud_segmentation/SegmentedPointCloud) - This sevice takes a point-cloud as input and returns a segmented point-cloud. The custom return message contains the original point-cloud color coded according to the segments, a list of the segmented point-clouds, and a list of indices representing which points belong to which segment.

`point_cloud_segmentation/change_segmentation_method` (input: String, output: bool) - This service lets the user change the segmentation method at run-time. It takes as input a string that should be one of the following plane, eucledian, region, or superclustering and returns a boolean indicating if the change was successful or not.

`point_cloud_segmentation/update_base_params` - This service lets the user update the `x_cutoff`, `y_cutoff`, and `z_cutoff` parameters.

`point_cloud_segmentation/update_base_params` - This service lets the user update the `x_cutoff`, `y_cutoff`, and `z_cutoff` parameters.

`point_cloud_segmentation/update_params` - This service lets the user update the parameters of the currently used segmentation algorithm.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
