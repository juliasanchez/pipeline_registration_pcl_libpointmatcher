template< typename points>
correspondences_search<  points>::correspondences_search()
{
	descriptor_method=1;
	rejector_method=1;
	descriptor_radius=0.1f;
}


// SET FUNCTIONS
//____________________________________________________________________________________________________________________________________________________________
template<typename points>
void correspondences_search<points>::setDescriptorMethod(int selected_method)
{
	descriptor_method=selected_method;
	std::cout<<"Descriptor : ";
	switch (descriptor_method)
	{
		case 1:
			{std::cout<<"FPFH method"<<std::endl<<std::endl;
			break;}
		case 2:
			{std::cout<<"3DSC method"<<std::endl<<std::endl;
			break;}
		case 3:
			{std::cout<<"USC method"<<std::endl<<std::endl;
			break;}
		case 4:
			{std::cout<<"SHOT method"<<std::endl<<std::endl;
			break;}
		default:
			{std::cout<<"method doesn't exist"<<std::endl<<std::endl;
			descriptor_method=1;}
	}
}


template<typename points>
void correspondences_search<points>::setRejectorMethod(int selected_method)
{
	rejector_method=selected_method;
	std::cout<<"Rejector : ";
	switch (rejector_method)
	{
		case 1:
			{std::cout<<"Sampleconsensus"<<std::endl<<std::endl;
			break;}
		case 2:
			{std::cout<<"Median_distance"<<std::endl<<std::endl;
			break;}
		case 3:
			{std::cout<<"One to one"<<std::endl<<std::endl;
			break;}
		case 4:
			{std::cout<<"Normal compatibility"<<std::endl<<std::endl;
			break;}
		case 5:
			{std::cout<<"Boundary Points"<<std::endl<<std::endl;
			break;}
		default:
			{std::cout<<"method doesn't exist"<<std::endl<<std::endl;
			rejector_method=1;}
	}
}


template<typename points>
void correspondences_search<  points>::setInputClouds(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target)
{
	src_cloud=source;
	tgt_cloud=target;
}

template<typename points>
void correspondences_search<  points>::setInputKeypoints(typename pcl::PointCloud<points>::Ptr &source, typename pcl::PointCloud<points>::Ptr &target)
{
	src_keypoints=source;
	tgt_keypoints=target;
}

template<typename points>
void correspondences_search<  points>::setInputNormals(typename pcl::PointCloud<pcl::Normal>::Ptr &source, typename pcl::PointCloud<pcl::Normal>::Ptr &target)
{
	src_normals=source;
	tgt_normals=target;
}

template<typename points>
void correspondences_search<  points>::setTrees(typename pcl::search::KdTree<points>::Ptr &src_treein, typename pcl::search::KdTree<points>::Ptr &tgt_treein)
{
	src_tree=src_treein;
	tgt_tree=tgt_treein;
}

template< typename points>
void correspondences_search<  points>::setDescriptorRadius(float descriptor_radiusin)
{
	descriptor_radius=descriptor_radiusin;
}

//GET FUNCTIONS
//________________________________________________________________________________________________________________________________________________________________________________________________________________

template<typename points>
template<typename descriptor_points>
void correspondences_search<points>::getFeatures(typename pcl::PointCloud<descriptor_points>* src_descriptor, typename pcl::PointCloud<descriptor_points>* tgt_descriptor)
{
	//std::cout<<"Computing descriptors"<<std::endl<<std::endl;
	switch (descriptor_method)
	{

		case 1:   // FPFH
		{
			pcl::PointCloud<pcl::FPFHSignature33>* src_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::FPFHSignature33>* >( src_descriptor );
			pcl::PointCloud<pcl::FPFHSignature33>* tgt_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::FPFHSignature33>* >( tgt_descriptor );

			getFeature_fpfh(src_cloud, src_keypoints, src_normals, src_tree, src_descriptor_);
			getFeature_fpfh(tgt_cloud, tgt_keypoints, tgt_normals,tgt_tree, tgt_descriptor_);
			break;
		}

		case 2:  //3DSC
		{
			pcl::PointCloud<pcl::ShapeContext1980>* src_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::ShapeContext1980>* >( src_descriptor );
			pcl::PointCloud<pcl::ShapeContext1980>* tgt_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::ShapeContext1980>* >( tgt_descriptor );

			getFeature_3dsc(src_cloud, src_keypoints, src_normals, src_tree, src_descriptor_);
			getFeature_3dsc(tgt_cloud, tgt_keypoints, tgt_normals, tgt_tree, tgt_descriptor_);
			break;
		}
		case 3:  //USC
		{
			pcl::PointCloud<pcl::UniqueShapeContext1960>* src_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::UniqueShapeContext1960>* >( src_descriptor );
			pcl::PointCloud<pcl::UniqueShapeContext1960>* tgt_descriptor_ = dynamic_cast< pcl::PointCloud<pcl::UniqueShapeContext1960>* >( tgt_descriptor );

			getFeature_usc(src_cloud, src_keypoints, src_tree, src_descriptor_);
		  getFeature_usc(tgt_cloud, tgt_keypoints, tgt_tree, tgt_descriptor_);

			break;
		}

	}
}

template<typename points>
template<typename descriptor_points>
void correspondences_search<points>::DetermineCorrespondences()
{
		std::cout<<"getting correspondences"<<std::endl<<std::endl;
			pcl::PointCloud<descriptor_points> src_descriptor;
			pcl::PointCloud<descriptor_points> tgt_descriptor;
			getFeatures<descriptor_points>(&src_descriptor, &tgt_descriptor);

			typename pcl::PointCloud<descriptor_points>::Ptr src_descriptor_ptr(new pcl::PointCloud<descriptor_points>);
			typename pcl::PointCloud<descriptor_points>::Ptr tgt_descriptor_ptr(new pcl::PointCloud<descriptor_points>);
			*src_descriptor_ptr=src_descriptor;
			*tgt_descriptor_ptr=tgt_descriptor;
			pcl::registration::CorrespondenceEstimation<descriptor_points, descriptor_points> est;
                        est.setInputSource(src_descriptor_ptr);
			est.setInputTarget(tgt_descriptor_ptr);
			est.determineCorrespondences(correspondences);
}

template<typename points>
void correspondences_search< points>::reject()
{

	switch (rejector_method)
	{
		case 1:  //sample_consensus
		{
			reject_simple_consensus(&correspondences);
			break;
		}

		case 2: //median
		{
			reject_median_distance(&correspondences);
			break;
		}
		case 3 :
		{
			reject_one_to_one(&correspondences);
			break;
		}
		case 4: //normal_compatibility
		{
			reject_normals_compatibility(&correspondences);
			break;
		}
		// case 5: //Boundaries
		// {
		// 	rejector.setMedianFactor(4.0); //Points with distance greater than median times factor will be rejected
		// 	break;
		// }
	}
}


template<typename points>
void correspondences_search< points>::reject_simple_consensus(pcl::Correspondences* correspondencesout)
{
	pcl::registration::CorrespondenceRejectorSampleConsensus<points> rejector;
	rejector.setInputSource(src_keypoints);
	rejector.setInputTarget(tgt_keypoints);
	rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
	rejector.setInlierThreshold(0.05);
	rejector.setMaximumIterations(5000);
	rejector.getCorrespondences(*correspondencesout);
}

template<typename points>
void correspondences_search< points>::reject_median_distance(pcl::Correspondences* correspondencesout)
{
	pcl::registration::CorrespondenceRejectorMedianDistance rejector;
	rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
	rejector.setMedianFactor(4.0); //Points with distance greater than median times factor will be rejected
	rejector.getCorrespondences(*correspondencesout);
}

template<typename points>
void correspondences_search< points>::reject_one_to_one(pcl::Correspondences* correspondencesout)
{
	pcl::registration::CorrespondenceRejectorOneToOne rejector;
	rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
	rejector.getCorrespondences(*correspondencesout);
}

template<typename points>
void correspondences_search< points>::reject_normals_compatibility(pcl::Correspondences* correspondencesout)
{
	pcl::registration::CorrespondenceRejectorSurfaceNormal rejector;
	rejector.setThreshold (0.5);  //max cosine(angle) between the normals for correspondence rejection : here 60°
	rejector.initializeDataContainer<points, pcl::Normal>();
	rejector.setInputSource <points> (src_cloud);
	rejector.setInputTarget <points> (tgt_cloud);
	rejector.setInputNormals<points, pcl::Normal>(src_normals);
	rejector.setTargetNormals<points, pcl::Normal>(tgt_normals);
	rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
	rejector.getCorrespondences(*correspondencesout);
}

template<typename points>
void correspondences_search< points>::getCorrespondences(pcl::Correspondences* correspondencesout)
{
	*correspondencesout=correspondences;
}

template<typename points>
void correspondences_search< points>::getFeature_fpfh(typename pcl::PointCloud<points>::Ptr cloudin, typename pcl::PointCloud<points>::Ptr keypoints, typename pcl::PointCloud<pcl::Normal>::Ptr normals,typename pcl::search::KdTree<points>::Ptr tree, typename pcl::PointCloud<pcl::FPFHSignature33>* features)
{
	pcl::FPFHEstimationOMP<points, pcl::Normal, pcl::FPFHSignature33> feature_estimation;

	feature_estimation.setSearchMethod(tree);
	feature_estimation.setRadiusSearch(descriptor_radius); //10 cm recherche des voisins pour faire le descriptor
	feature_estimation.setSearchSurface(cloudin);
	feature_estimation.setInputCloud(keypoints);
	feature_estimation.setInputNormals(normals);
	feature_estimation.compute(*features);
}

template<typename points>
void correspondences_search< points>::getFeature_usc(typename pcl::PointCloud<points>::Ptr cloudin, typename pcl::PointCloud<points>::Ptr keypoints,typename pcl::search::KdTree<points>::Ptr tree, typename pcl::PointCloud<pcl::UniqueShapeContext1960>* features)
{
	pcl::UniqueShapeContext<points, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> feature_estimation;
	feature_estimation.setSearchMethod(tree);
	feature_estimation.setRadiusSearch(descriptor_radius); //5 cm recherche des voisins pour faire le descriptor
	feature_estimation.setInputCloud(keypoints);
	feature_estimation.setSearchSurface(cloudin);
	feature_estimation.setRadiusSearch(descriptor_radius); // neigbours to take into account
	feature_estimation.setMinimalRadius(descriptor_radius / 10.0); //to avoid being too sensitive in bins close to center
	feature_estimation.setPointDensityRadius(descriptor_radius / 5.0); //radius for local density
	feature_estimation.setLocalRadius(descriptor_radius); //radius for local frame
	feature_estimation.compute(*features);
}

template<typename points>
void correspondences_search< points>::getFeature_3dsc(typename pcl::PointCloud<points>::Ptr cloudin, typename pcl::PointCloud<points>::Ptr keypoints, typename pcl::PointCloud<pcl::Normal>::Ptr normals, typename pcl::search::KdTree<points>::Ptr tree, typename pcl::PointCloud<pcl::ShapeContext1980>* features)
{
	pcl::ShapeContext3DEstimation<points, pcl::Normal, pcl::ShapeContext1980> feature_estimation;
	feature_estimation.setSearchMethod(tree);

	feature_estimation.setSearchSurface(cloudin);
	feature_estimation.setInputCloud(keypoints);
	//feature_estimation.setInputCloud(cloudin);
	feature_estimation.setInputNormals(normals);

	feature_estimation.setMinimalRadius(descriptor_radius / 10.0);
	feature_estimation.setRadiusSearch(descriptor_radius);
	feature_estimation.setPointDensityRadius(descriptor_radius/ 5.0);
	feature_estimation.compute(*features);
}
