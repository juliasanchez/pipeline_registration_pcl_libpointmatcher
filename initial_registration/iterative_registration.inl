template<typename points>
iterative_registration<points>::iterative_registration()
{
}

template<typename points>
void iterative_registration<points>::setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target)
{
	src_cloud=source;
	tgt_cloud=target;
}

template<typename points>
void iterative_registration<points>::set_parameters(float d_max_in, float ransac_thresh_in, float eps_in, int it_max_in)
{
	d_max=d_max_in;
	ransac_thresh=ransac_thresh_in;
	eps=eps_in;
	it_max=it_max_in;
}


template<typename points>
void iterative_registration<points>::regist(typename pcl::PointCloud<points>::Ptr aligned)
{
	pcl::IterativeClosestPoint<points, points> icp;
	icp.setMaxCorrespondenceDistance(d_max);
	icp.setRANSACOutlierRejectionThreshold(ransac_thresh);
	icp.setTransformationEpsilon(eps);
	icp.setMaximumIterations(it_max);
	icp.setInputSource(src_cloud);
	icp.setInputTarget(tgt_cloud);
	icp.align(*src_cloud);
	transformation = icp.getFinalTransformation();
	transformPointCloud(*src_cloud, *aligned, transformation);
}

template<typename points>
void iterative_registration<points>::getTransfo(Eigen::Matrix4f* &transfo)
{
	transfo=&transformation;
}
