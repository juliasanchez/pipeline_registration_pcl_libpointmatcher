#ifndef DISPLAY_HIST
#define DISPLAY_HIST

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include<pcl/visualization/histogram_visualizer.h>

void display_hist (pcl::PointCloud<pcl::FPFHSignature33 >* features);

#include "display_hist.inl"

#endif // DISPLAY_HIST
