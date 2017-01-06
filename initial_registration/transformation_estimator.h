#ifndef TRANSFORMATION_ESTIMATOR_H
#define TRANSFORMATION_ESTIMATOR_H

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>

template<typename points>
class transformation_estimator
{
public:
	transformation_estimator();
	void setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target);
	void setInputCorrespondences(pcl::Correspondences*);
	void getTransfo(Eigen::Matrix4f* &transfo);
	void getTransformedPointcloud(typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<points>::Ptr);



private:
	typename pcl::PointCloud<points>::Ptr src_cloud;
	typename pcl::PointCloud<points>::Ptr tgt_cloud;
	Eigen::Matrix4f transformation;
	pcl::Correspondences* correspondences;

};

#include "transformation_estimator.inl"

#endif // transformation_estimator
