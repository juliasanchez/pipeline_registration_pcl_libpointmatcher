template<class points>
keypoints_search<points>::keypoints_search()
{
	method=1;
	sigma0=0.01f;
	octaves=3;
	scales=2;
	resolution=0.003;
}

template<class points>
void keypoints_search<points>::setMethod(int selected_method)
{
	method=selected_method;
	std::cout<<"Keypoints : ";
	switch (method)
	{
		case 0:
			{std::cout<<"Harris method"<<std::endl<<std::endl;
			break;}
		case 1:
			{std::cout<<"SIFT method"<<std::endl<<std::endl;
			break;}
		case 2:
			{std::cout<<"ISS method"<<std::endl<<std::endl;
			break;}
		default:
			{std::cout<<"method doesn't exist"<<std::endl<<std::endl;
			method=1;}
	}

}

template<class points>
void keypoints_search<points>::setInputCloud(typename pcl::PointCloud<points>::Ptr &cloudin)
{
	//std::cout << "fun: cloudin.use_count() == " << cloudin.use_count() << std::endl;
	inputCloud=cloudin;
	//std::cout << "fun: cloudin.use_count() == " << cloudin.use_count() << std::endl;
}

template<class points>
void keypoints_search<points>::setInputNormals(typename pcl::PointCloud<pcl::Normal>::Ptr &normalsin)
{
	normals=normalsin;
}


template<class points>
void keypoints_search<points>::setCloudResolution(double reso)
{
	resolution=reso;
}

template<class points>
void keypoints_search<points>::setParameters(float sigma0in, int octavesin, int scalesin )
{
	sigma0=sigma0in;
	octaves=octavesin;
	scales=scalesin;
}

template<class points>
void keypoints_search<points>::setParameters(float gammain, int min_neighin, float rad )
{
	gamma=gammain;
	min_neigh=min_neighin;
        radius=rad;
}

template<class points>
void keypoints_search<points>::setTree(typename pcl::search::KdTree<points>::Ptr &treein)
{
	tree=treein;
}


template<class points>
void keypoints_search<points>::getKeypoints(typename pcl::PointCloud<points>::Ptr keypoints)
{

	switch (method)
	{
		/*case 0:
		{
			//typename pcl::PointCloud<pcl::PointXYZI>::Ptr convertedCloud (new pcl::PointCloud<pcl::PointXYZI>);
			//pcl::PointCloudXYZRGBtoXYZI (*&inputCloud, *&convertedCloud);

			pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris;
			harris.setRadius(0.1);
		  //harris.setInputCloud(convertedCloud);
			harris.setInputCloud(inputCloud);
			//typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_xyzi;
			//harris.compute(*keypoints_xyzi);
			typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints;
			harris.compute(*keypoints);

			/*keypoints->points.resize(keypoints_xyzi->size());
			size_t i = 0;
			for (size_t k = 0; k < inputCloud->points.size(); k++)
			{
		    if(inputCloud->points[k].x == keypoints_xyzi->points[i].x &&
		    	inputCloud->points[k].y == keypoints_xyzi->points[i].y &&
		    	inputCloud->points[k].z == keypoints_xyzi->points[i].z)
					{
						i++;
						keypoints->points[i]=inputCloud->points[k];
					}
			}*/

			//break;
		//}

		case 1:
		{
			pcl::SIFTKeypoint<points, points> sift;
                        sift.setSearchMethod(typename pcl::search::KdTree<points>::Ptr(new pcl::search::KdTree<points>));
                        //sift.setSearchMethod(tree);
                        //std::cout<<"sigma : "<<sigma0<<" octaves : "<<octaves<<" scales : "<<scales<<std::endl;
                        sift.setScales(sigma0, octaves, scales);
                        //sift.setScales(0.01f, 3, 2);  // sigma du plus petit floutage (1cm), nombre d'octaves, nombre de floutages dans chaque octave
                        sift.setMinimumContrast(0); // on enlève du keypoint avec pas assez de différences avec ses voisins

			sift.setInputCloud(inputCloud);
                        sift.compute(*keypoints);
			break;
		}
		case 2:
		{

			pcl::ISSKeypoint3D<points, points> iss;
			iss.setSearchMethod ( tree );
                        iss.setSalientRadius (radius); // about 6 points to compute scatter matrix
			iss.setMinNeighbors (min_neigh); //minimum number of neigbors to find in the sphere of radius SalientRadius
			iss.setNonMaxRadius (0.2f); // rayon pour prendre en compte les voisins dans la comparaison de lambda3 (on ne garde que le point avec le plus petit lambda3)
			iss.setNormals(normals);
			iss.setThreshold21(gamma);
			iss.setThreshold32(gamma);

                        iss.setBorderRadius (0.2f); // at wich point it belongs to border
			
		//	iss.setAngleThreshold (static_cast<float> (M_PI) / 3.0);
		//	iss.setNumberOfThreads (1);

			iss.setInputCloud (inputCloud);
			iss.compute (*keypoints);
			break;     
		}
	}

}
