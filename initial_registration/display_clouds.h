#ifndef DISPLAY_CLOUDS
#define DISPLAY_CLOUDS

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

void display_clouds (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, int* color1, int* color2, int size1, int size2);

#include "display_clouds.inl"

#endif // DISPLAY_CLOUDS
