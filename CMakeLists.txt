cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  pcl_msgs
  tf2
  tf2_ros
  supervoxel_clustering
)

find_package(PCL 1.8 REQUIRED)

add_message_files(
  FILES
  SegmentedPointCloud.msg
)

add_service_files(
  FILES
  SegmentScene.srv
  ChangeSegmentationMethod.srv
  ChangeSegmentationMethod.srv
  EucledianClusterParams.srv
  PlaneSegmentationParams.srv
  RegionGrowingSegParams.srv
  SuperClusteringSegParams.srv
  BaseSegmentationParams.srv
)

generate_messages(DEPENDENCIES sensor_msgs pcl_msgs)

catkin_package(CATKIN_DEPENDS message_runtime supervoxel_clustering)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(base_segmentation src/base_segmentation.cpp)
add_library(plane_segmentation src/plane_segmentation.cpp)
add_library(eucledian_distance_segmentation src/eucledian_distance_segmentation.cpp)
add_library(super_clustering_segmentation src/super_clustering_segmentation.cpp)
add_library(region_growing_segmentation src/region_growing_segmentation.cpp)

add_dependencies(base_segmentation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(plane_segmentation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(eucledian_distance_segmentation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(super_clustering_segmentation ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(region_growing_segmentation ${PROJECT_NAME}_generate_messages_cpp)

add_executable(main src/main.cpp)
target_link_libraries(main 
  base_segmentation
  plane_segmentation
  eucledian_distance_segmentation
  super_clustering_segmentation
  region_growing_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(main ${PROJECT_NAME}_generate_messages_cpp)


install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS main
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
