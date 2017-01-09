#ifndef DISPLAY_CORRESPONDENCES
#define DISPLAY_CORRESPONDENCES

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>

void display_correspondences (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, pcl::Correspondences Correspondences);

#include "display_correspondences.inl"

#endif // DISPLAY_CORRESPONDENCES
