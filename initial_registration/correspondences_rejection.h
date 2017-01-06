#ifndef CORRESPONDENCES_REJECTION_H
#define CORRESPONDENCES_REJECTION_H

#include <vector>
#include <string>
#include <iostream>

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

template<typename points>
class correspondences_rejection
{
public:
	correspondences_rejection();
	void setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target);
	void setInputCorrespondences(pcl::Correspondences*);
	void getCorrespondences(pcl::Correspondences*);

private:
	pcl::Correspondences* correspondencesin;
	typename pcl::PointCloud<points>::Ptr src_cloud;
	typename pcl::PointCloud<points>::Ptr tgt_cloud;
};

#include "correspondences_rejection.inl"

#endif // CORRESPONDENCES_REJECTION
