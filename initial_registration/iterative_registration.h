#ifndef ITERATIVE_REGISTRATION_H
#define ITERATIVE_REGISTRATION_H

#include <vector>
#include <string>
#include <iostream>

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

template<typename points>
class iterative_registration
{
public:
	iterative_registration();
	void setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target);
	void set_parameters(float d_max_in, float ransac_thresh_in, float eps_in, int it_max_in);
	void regist(typename pcl::PointCloud<points>::Ptr );
	void getTransfo(Eigen::Matrix4f* &transfo);

private:
	typename pcl::PointCloud<points>::Ptr src_cloud;
	typename pcl::PointCloud<points>::Ptr tgt_cloud;
	float d_max;
	float ransac_thresh;
	float eps;
	int it_max;
	Eigen::Matrix4f transformation;

};

#include "iterative_registration.inl"

#endif // ITERATIVE_REGISTRATION
