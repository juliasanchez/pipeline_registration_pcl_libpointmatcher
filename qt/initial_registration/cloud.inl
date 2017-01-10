template<typename points>
cloud<points>::cloud()
{
}

template<typename points>
void cloud<points>::getScale(float* Volume)
{
	points minPt, maxPt;
	pcl::getMinMax3D (*cloud_in, minPt, maxPt);
  float dist_x=maxPt.x-minPt.x;
  float dist_y=maxPt.y-minPt.y;
  float dist_z=maxPt.z-minPt.z;

	Eigen::Matrix4f scaling= Eigen::Matrix4f::Identity();
	//float resize=max(   (maxPt.x-minPt.x),   (maxPt.y-minPt.y),    (maxPt.z-minPt.z)     );
	//transformation*float(1/resize);
	*Volume= dist_x*dist_y*dist_z;
}


template<typename points>
void cloud<points>::setInputCloud(typename pcl::PointCloud<points>::Ptr &cloudin)
{
	cloud_in=cloudin;
}

template<typename points>
void cloud<points>::getInputCloud(typename pcl::PointCloud<points>::Ptr cloudout)
{
	cloudout=cloud_in;
}

template<typename points>
void cloud<points>::getNormals(float radius, typename pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	pcl::NormalEstimationOMP<points, pcl::Normal> normal_estimation;
	normal_estimation.setSearchMethod(typename pcl::search::KdTree<points>::Ptr(new pcl::search::KdTree<points>));
	normal_estimation.setRadiusSearch(radius);

	normal_estimation.setInputCloud(cloud_in);
	normal_estimation.compute(*normals);
}

template<typename points>
void cloud<points>::getTree(typename pcl::search::KdTree<points>::Ptr &tree_out)
{
	tree.setInputCloud(cloud_in);
	*tree_out=tree;
}

template<typename points>
void cloud<points>::sample()
{
    pcl::UniformSampling<points> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_in);
    uniform_sampling.setRadiusSearch (0.03);
    std::cout << "before : " << cloud_in->size ()<<std::endl;
    uniform_sampling.filter (*cloud_in);
    std::cout << " after sampling : " << cloud_in->size () << std::endl<<std::endl;
}

template<typename points>
void cloud<points>::getSize(int *size)
{
	*size=cloud_in->width*cloud_in->height;
}

template<typename points>
void cloud<points>::load(std::string pcd_file)
{
	if( pcl::io::loadPCDFile<points>( pcd_file, *cloud_in ) == -1 )
	{
	    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}
}

template<typename points>
void cloud<points>::clean()
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
//    pcl::StatisticalOutlierRemoval<points> sor;
//    sor.setInputCloud (cloud_in);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*cloud_in);

    typename pcl::ConditionAnd<points>::Ptr condition (new pcl::ConditionAnd<points>);
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("x", pcl::ComparisonOps::LT, 3)));
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("y", pcl::ComparisonOps::LT, 3)));
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("z", pcl::ComparisonOps::LT, 3)));
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("x", pcl::ComparisonOps::GT, -3)));
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("y", pcl::ComparisonOps::GT, -3)));
    condition->addComparison(typename pcl::FieldComparison<points>::ConstPtr(new typename pcl::FieldComparison<points>("z", pcl::ComparisonOps::GT, -3)));

    pcl::ConditionalRemoval<points> filter (condition);
    filter.setInputCloud(cloud_in);
    filter.filter(*cloud_in);

    pcl::PassThrough<points> pass;
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0,0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (15,15);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_in);
}

template<typename points>
double cloud<points>::computeCloudResolution ()
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);

  for (size_t i = 0; i < cloud_in->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud_in)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}
