#ifndef DISPLAY_NORMALS
#define DISPLAY_NORMALS

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

void display_normals (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::Normal>::Ptr normals);

#include "display_normals.inl"

#endif // DISPLAY_CLOUDS
