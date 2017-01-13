#ifndef GET_COST_MATRIX
#define GET_COST_MATRIX

#include <pcl/features/fpfh.h>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>

void get_cost_matrix(pcl::PointCloud<pcl::FPFHSignature33> features1, pcl::PointCloud<pcl::FPFHSignature33> features2, dlib::matrix<float>* cost);

#include "get_cost_matrix.inl"

#endif // GET_COST_MATRIX
