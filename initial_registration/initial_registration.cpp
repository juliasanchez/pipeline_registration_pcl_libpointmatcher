//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "display_clouds.h"
#include "keypoints_search.h"
#include "correspondences_search.h"
#include "iterative_registration.h"
#include "cloud.h"
#include "transformation_estimator.h"
#include <pcl/registration/transformation_validation_euclidean.h>

// PCL INCLUDES

using pcl::transformPointCloud;

//typedef pcl::PointXYZRGB pcl_point;
typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;

//...............................................................................................................................

int main(int argc, char** argv)
{
//std::cout<<"beginning"<<std::endl<<std::endl;
	// if ( argc < 9 && argc !=1 )
	// {
	// 	std::cout << "Usage: ./first_visu2 filename1.pcd filename2.pcd  KeypointsMethod{0,2} -->2 parameters for iss & 3 for sift, DescriptorMethod{1,2,3,4} DescriptorRadius RejectorMethod{1,2,3,4} "
	// 			    << std::endl
	// 			    << std::endl;
	// 	exit(EXIT_FAILURE);
	// }
	//
	// if ( argc > 14 )
	// {
	// 	std::cout << "TOO MANY ARGUMENTS"
	// 					<< std::endl
	// 					<< std::endl;
	// 	exit(EXIT_FAILURE);
	// }

	std::string pcd_file1( argv[1] );
	std::string pcd_file2( argv[2] );

//class cloud implemented to get normals, size and to load and display the cloud
	pcl::PointCloud<pcl_point>::Ptr cloud1(new pcl::PointCloud<pcl_point>);
	pcl::PointCloud<pcl_point>::Ptr cloud2(new pcl::PointCloud<pcl_point>);

	cloud<pcl_point> cloud_src;
	cloud_src.setInputCloud(cloud1);
	cloud_src.load(pcd_file1);

	cloud<pcl_point> cloud_tgt;
	cloud_tgt.setInputCloud(cloud2);
	cloud_tgt.load(pcd_file2);
  int color1[3]={20,230,20};
  int color2[3]={20,20,230};
	//display_clouds(cloud1, cloud2, color1,color2);
	//display_clouds(cloud1, cloud2, color1,color2);

// //cleaning and get important features
// 	float Vol1, Vol2;
// 	cloud_src.clean();
// 	pcl::search::KdTree<pcl_point>::Ptr tree1 (new pcl::search::KdTree<pcl_point>);
// 	cloud_src.getTree(tree1);
// 	cloud_src.getScale(&Vol1);
// 	double resolution1=cloud_src.computeCloudResolution();
//
// 	cloud_tgt.clean();
// 	typename pcl::search::KdTree<pcl_point>::Ptr tree2 (new pcl::search::KdTree<pcl_point>);
// 	cloud_tgt.getTree(tree2);
// 	cloud_tgt.getScale(&Vol2);
// 	double resolution2=cloud_tgt.computeCloudResolution();
//
// 	display_clouds(cloud1, cloud2, color1,color2);



	pcl::visualization::PCLVisualizer viewer1 ("1");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud1_color (cloud1, color1[1], color1[2], color1[3]);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud2_color (cloud2, color2[1], color2[2], color2[3]);
  // We add the point cloud to the viewer and pass the color handler
  viewer1.addPointCloud (cloud1, cloud1_color, "cloud1");
  viewer1.addPointCloud (cloud2, cloud2_color, "cloud2");
  viewer1.addCoordinateSystem (1.0, "cloud1", 0);
  viewer1.setBackgroundColor(0.05, 0.05, 0.05, 0); // dark grey
  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
  while (!viewer1.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer1.spinOnce ();
  }



	pcl::visualization::PCLVisualizer viewer2 ("2");
	// We add the point cloud to the viewer and pass the color handler
	viewer2.addPointCloud (cloud1, cloud1_color, "cloud1");
	viewer2.addPointCloud (cloud2, cloud2_color, "cloud2");
	viewer2.addCoordinateSystem (1.0, "cloud1", 0);
	viewer2.setBackgroundColor(0.05, 0.05, 0.05, 0); // dark grey
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
	viewer2.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
	while (!viewer2.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer2.spinOnce ();
	}



//
// 	std::cout<<"Volumes : "<<Vol1<<"   "<<Vol2<<std::endl<<std::endl;
// 	std::cout<<"resolutions : "<<resolution1<<"   "<<resolution2<<std::endl<<std::endl;
//
// 	//display_clouds(cloud1, cloud2, Color::Green,Color::Red);

//
// 	//std::cout<<"Computing normals"<<std::endl<<std::endl;
// 	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
// 	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
// 	cloud_src.getNormals(0.1, normals1);
// 	cloud_tgt.getNormals(0.1, normals2);
//
// //searching keypoints
//
// 	//std::cout<<"Determining keypoints"<<std::endl<<std::endl;
// 	pcl::PointCloud<pcl_point>::Ptr keypoints1 (new pcl::PointCloud<pcl_point>);
// 	pcl::PointCloud<pcl_point>::Ptr keypoints2 (new pcl::PointCloud<pcl_point>);
//
// 	keypoints_search<pcl_point> ks;
// 	int n=3;
// 	ks.setMethod(atoi(argv[n]));
// 	if(atoi(argv[n])==1) //SIFT
// 	{
// 		ks.setParameters(atof(argv[n+1]), atoi(argv[n+2]), atoi(argv[n+3]));
// 		n=n+4;
// 	}
// 	else if(atoi(argv[n])==2)//ISS
// 	{
// 		ks.setParameters(atof(argv[n+1]),atoi(argv[n+2]));
// 		n=n+3;
// 	}
//
// 	ks.setTree(tree1);
// 	ks.setInputCloud(cloud1);
// 	ks.setInputNormals(normals1);
// 	ks.setCloudResolution(resolution1);
// 	ks.getKeypoints(keypoints1);
//
// 	ks.setTree(tree2);
// 	ks.setInputCloud(cloud2);
// 	ks.setInputNormals(normals2);
// 	ks.setCloudResolution(resolution2);
// 	ks.getKeypoints(keypoints2);
//
//   std::cerr << "Number of points " << cloud1->width * cloud1->height << " data points." << std::endl;
// 	std::cerr << "Number of keypoints " << keypoints1->width * keypoints1->height << " data points." << std::endl;
//
// 	display_clouds(cloud1, keypoints1, Color::Black, Color::Red);
// 	//display_clouds(cloud2, keypoints2, Color::Black, Color::Red);
//
// //Derive correspondences
// 	//std::cout<<"Deriving correspondences"<<std::endl<<std::endl;
// 	pcl::Correspondences correspondences;
// 	correspondences_search<pcl_point> cs;
// 	cs.setInputClouds(cloud1, cloud2);
// 	cs.setInputNormals(normals1, normals2);
// 	cs.setTrees(tree1, tree2);
// 	cs.setInputKeypoints(keypoints1,keypoints2);
// 	int des_method =atoi(argv[n]);
// 	cs.setDescriptorMethod(des_method);
// 	cs.setDescriptorRadius(atof(argv[n+1]));
//
// 	//cs.getFeatures();
//
// 	switch(des_method)
// 	{
// 		case 1: //FPFH
// 		{
// 			cs.DetermineCorrespondences<typename pcl::FPFHSignature33>();
// 			break;
// 		}
// 		case 2: //3DSC //case 4:
// 		{
// 			cs.DetermineCorrespondences<typename pcl::ShapeContext1980>();
// 			break;
// 		}
// 		case 3:  //USC
// 		{
// 			cs.DetermineCorrespondences<typename pcl::UniqueShapeContext1960>();
// 			break;
// 		}
// 		default:
// 		{
// 			cs.DetermineCorrespondences<typename pcl::FPFHSignature33>();
// 			break;
// 		}
// 	}
// 	n=n+2;
//
// 	for (int i=n; i<argc; i++)
// 	{
// 		int rej_method=atoi(argv[i]);
// 		cs.setRejectorMethod(rej_method);
// 		cs.reject();
// 	}
// 	cs.getCorrespondences(&correspondences);
// //Get transformation and transform src to tgt
//
// 	//std::cout<<"Getting transformation"<<std::endl<<std::endl;
// 	transformation_estimator<pcl_point> te;
// 	if (des_method==0)
// 	{
// 		te.setInputClouds(cloud1, cloud2);
// 	}
// 	else
// 	{
// 		te.setInputClouds(keypoints1, keypoints2);
// 	}
// 	te.setInputCorrespondences(&correspondences);
// 	Eigen::Matrix4f* initial_transfo;
// 	te.getTransfo(initial_transfo);
// 	pcl::PointCloud<pcl_point>::Ptr pre_aligned (new pcl::PointCloud<pcl_point>);
// 	te.getTransformedPointcloud(cloud1, pre_aligned);
// 	display_clouds(pre_aligned, cloud2, Color::Green, Color::Red);
//
//
// //ICP REGISTRATION
// 	iterative_registration<pcl_point> icp;
// 	icp.setInputClouds(pre_aligned, cloud2);
// 	icp.set_parameters(0.5f, 0.05f,0.05f, 600);
// 	pcl::PointCloud<pcl_point>::Ptr aligned (new pcl::PointCloud<pcl_point>);
// 	icp.regist(aligned);
// 	Eigen::Matrix4f* icp_transfo;
// 	icp.getTransfo(icp_transfo);
//
// 	//display_clouds(aligned, cloud2, Color::Green, Color::Red);
//
// 	//std::cout<<"icp : "<<*icp_transfo<<std::endl<<std::endl;
// 	//std::cout<<"initial : "<<*initial_transfo<<std::endl<<std::endl;
// 	Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
// 	pcl::registration::TransformationValidationEuclidean<pcl_point, pcl_point> validation_before;
// 	double score_without_overlapping= validation_before.validateTransformation (cloud1, cloud2, id);
// 	std::cout<<score_without_overlapping;
//
// 	Eigen::Matrix4f transformation_matrix=(*icp_transfo)*(*initial_transfo);
// 	pcl::registration::TransformationValidationEuclidean<pcl_point, pcl_point> validation;
// 	double score= validation.validateTransformation (cloud1, cloud2, transformation_matrix);
// 	std::cout<<","<<score<<std::endl<<std::endl;

	return 0;
}
