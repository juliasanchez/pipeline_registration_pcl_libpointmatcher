#ifndef GET_COST_MATRIX_FPFH
#define GET_COST_MATRIX_FPFH

#include <pcl/features/fpfh.h>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>

void get_cost_matrix_fpfh(pcl::PointCloud<pcl::FPFHSignature33> features1, pcl::PointCloud<pcl::FPFHSignature33> features2, dlib::matrix<int>* cost_matrix);

#include "get_cost_matrix_fpfh.inl"

#endif // GET_COST_MATRIX_FPFH
