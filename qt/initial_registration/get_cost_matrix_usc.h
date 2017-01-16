#ifndef GET_COST_MATRIX_USC
#define GET_COST_MATRIX_USC

#include <pcl/features/usc.h>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>

void get_cost_matrix_usc(pcl::PointCloud<pcl::UniqueShapeContext1960> features1, pcl::PointCloud<pcl::UniqueShapeContext1960> features2, dlib::matrix<int>* cost_matrix);

#include "get_cost_matrix_usc.inl"

#endif // GET_COST_MATRIX_USC
