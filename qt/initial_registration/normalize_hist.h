#ifndef NORMALIZE_HIST
#define NORMALIZE_HIST

#include <pcl/features/fpfh.h>

void normalize_hist(pcl::PointCloud<pcl::FPFHSignature33>* features);

#include "normalize_hist.inl"

#endif // NORMALIZE_HIST
