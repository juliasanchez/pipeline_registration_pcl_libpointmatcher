#ifndef CORRESPONDENCES_SEARCH_H
#define CORRESPONDENCES_SEARCH_H

// PCL INCLUDES

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/shot.h>
#include <pcl/features/usc.h>
#include <pcl/features/3dsc.h>
#include <pcl/search/kdtree.h>

#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>



template<typename points>
class correspondences_search
{
public:
	correspondences_search();
	void setDescriptorMethod(int);
	void setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target);
	void setInputKeypoints(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target);
	void setInputNormals(typename pcl::PointCloud<pcl::Normal>::Ptr &source, typename pcl::PointCloud<pcl::Normal>::Ptr &target);
	void setTrees(typename pcl::search::KdTree<points>::Ptr&,typename pcl::search::KdTree<points>::Ptr&);
	void setDescriptorRadius(float);
	void setRejectorMethod(int);
	void getCorrespondences(pcl::Correspondences*);

  template<typename descriptor_points> void DetermineCorrespondences();
	void reject();

private:
	void getFeature_fpfh(typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<pcl::Normal>::Ptr, typename pcl::search::KdTree<points>::Ptr, typename pcl::PointCloud<pcl::FPFHSignature33>*);
	void getFeature_3dsc(typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<pcl::Normal>::Ptr, typename pcl::search::KdTree<points>::Ptr, typename pcl::PointCloud<pcl::ShapeContext1980>*);
	void getFeature_usc(typename pcl::PointCloud<points>::Ptr, typename pcl::PointCloud<points>::Ptr, typename pcl::search::KdTree<points>::Ptr, typename pcl::PointCloud<pcl::UniqueShapeContext1960>*);
	void reject_simple_consensus(pcl::Correspondences* correspondencesout);
	void reject_median_distance(pcl::Correspondences* correspondencesout);
	void reject_one_to_one(pcl::Correspondences* correspondencesout);
	void reject_normals_compatibility(pcl::Correspondences* correspondencesout);
	template<typename descriptor_points>void getFeatures(typename pcl::PointCloud<descriptor_points>*, typename pcl::PointCloud<descriptor_points>*);
	typename pcl::PointCloud<pcl::Normal>::Ptr tgt_normals;
	typename pcl::PointCloud<pcl::Normal>::Ptr src_normals;
	typename pcl::PointCloud<points>::Ptr src_cloud;
	typename pcl::PointCloud<points>::Ptr tgt_cloud;
	typename pcl::PointCloud<points>::Ptr src_keypoints;
	typename pcl::PointCloud<points>::Ptr tgt_keypoints;
	int descriptor_method;
	int rejector_method;
	typename pcl::search::KdTree<points>::Ptr src_tree;
	typename pcl::search::KdTree<points>::Ptr tgt_tree;
	float descriptor_radius;
	pcl::Correspondences correspondences;

};

#include "correspondences_search.inl"

#endif // CORRESPONDENCES_SEARCH
