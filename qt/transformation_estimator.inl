template<typename points>
transformation_estimator<points>::transformation_estimator()
{
}

template<typename points>
void transformation_estimator<points>::setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target)
{
	src_cloud=source;
	tgt_cloud=target;
}

template<typename points>
void transformation_estimator<points>::getTransformedPointcloud(typename pcl::PointCloud<points>::Ptr cloud_in, typename pcl::PointCloud<points>::Ptr cloud_out)
{
	transformPointCloud(*cloud_in, *cloud_out, transformation);
}

template<typename points>
void transformation_estimator<points>::getTransfo(Eigen::Matrix4f* &transfo)
{
	pcl::registration::TransformationEstimationSVD<points, points> estimator;
	estimator.estimateRigidTransformation(*src_cloud, *tgt_cloud, *correspondences, transformation);
	transfo=&transformation;
}


template<typename points>
void transformation_estimator<points>::setInputCorrespondences(pcl::Correspondences* correspondences_in)
{
	correspondences=correspondences_in;
}
