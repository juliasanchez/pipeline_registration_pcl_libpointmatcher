#ifndef KEYPOINTS_SEARCH_H
#define KEYPOINTS_SEARCH_H

#include <vector>
#include <string>
#include <iostream>

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/extract_indices.h>


//AJOUTER NARF+ HARRIS + others

template<class points>
class keypoints_search
{
public:
	keypoints_search();
  void setInputCloud(typename pcl::PointCloud<points>::Ptr&);
	void setCloudResolution(double);
  void setMethod(int);
  void getKeypoints(typename pcl::PointCloud<points>::Ptr);
	void setInputNormals(typename pcl::PointCloud<pcl::Normal>::Ptr&);
	void setParameters(float,int,int);
	void setParameters(float, int);
	void setTree(typename pcl::search::KdTree<points>::Ptr&);

private:
  int method;
	typename pcl::PointCloud<points>::Ptr inputCloud;
	float NonMaximumRadius;
	double resolution;
	typename pcl::PointCloud<pcl::Normal>::Ptr normals;
	float sigma0;
	int octaves;
	int scales;
	float gamma;
	int min_neigh;
	typename pcl::search::KdTree<points>::Ptr tree;
};

#include "keypoints_search.inl"

#endif // KEYPOINTS_SEARCH
