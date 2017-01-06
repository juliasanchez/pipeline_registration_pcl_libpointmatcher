template<typename points>
correspondences_rejection<points>::correspondences_rejection()
{
}

template<typename points>
void correspondences_rejection<points>::setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target)
{
	src_cloud=source;
	tgt_cloud=target;
}

template<typename points>
void correspondences_rejection<points>::setInputCorrespondences(pcl::Correspondences* correspondences)
{
	correspondencesin=correspondences;
}

template<typename points>
void correspondences_rejection<points>::getCorrespondences(pcl::Correspondences* correspondencesout)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<points> rejector;
	rejector.setInputSource(src_cloud);
	rejector.setInputTarget(tgt_cloud);
	rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(*correspondencesin));
	rejector.setInlierThreshold(0.05);
	rejector.setMaximumIterations(5000);
	rejector.getCorrespondences(*correspondencesout);
}
