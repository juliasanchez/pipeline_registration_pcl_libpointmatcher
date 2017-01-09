#ifndef DISPLAY_CORRESPONDENCES_WITH_CLOUDS
#define DISPLAY_CORRESPONDENCES_WITH_CLOUDS

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>

void display_correspondences_with_clouds (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints2, pcl::Correspondences correspondences);

#include "display_correspondences_with_clouds.inl"

#endif // DISPLAY_CORRESPONDENCES_WITH_CLOUDS
