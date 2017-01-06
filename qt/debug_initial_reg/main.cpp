//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "display_clouds.h"
#include "display_normals.h"
#include "display_correspondences.h"
#include "display_correspondences_with_clouds.h"
#include "keypoints_search.h"
#include "transformation_estimator.h"
//#include "correspondences_search.h"
#include "cloud.h"
//#include "transformation_estimator.h"
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>


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
       // display_clouds(cloud1, cloud2, color1, color2, 1, 1);

 //cleaning and get important features
        float Vol1, Vol2;
        cloud_src.clean();
        cloud_src.sample();
        pcl::search::KdTree<pcl_point>::Ptr tree1 (new pcl::search::KdTree<pcl_point>);
        cloud_src.getTree(tree1);
        cloud_src.getScale(&Vol1);
        double resolution1=cloud_src.computeCloudResolution();

        cloud_tgt.clean();
        cloud_tgt.sample();
        typename pcl::search::KdTree<pcl_point>::Ptr tree2 (new pcl::search::KdTree<pcl_point>);
        cloud_tgt.getTree(tree2);
        cloud_tgt.getScale(&Vol2);
        double resolution2=cloud_tgt.computeCloudResolution();

        //display_clouds(cloud1, cloud2, color1, color2, 1, 1);


        std::cout<<"Volumes : "<<Vol1<<"   "<<Vol2<<std::endl<<std::endl;
        std::cout<<"resolutions : "<<resolution1<<"   "<<resolution2<<std::endl<<std::endl;

        //std::cout<<"Computing normals"<<std::endl<<std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
        cloud_src.getNormals(0.2, normals1);
        cloud_tgt.getNormals(0.2, normals2);

        //display_normals(cloud1,normals1);
        //display_normals(cloud2,normals2);

 //searching keypoints

        //std::cout<<"Determining keypoints"<<std::endl<<std::endl;
        pcl::PointCloud<pcl_point>::Ptr keypoints1 (new pcl::PointCloud<pcl_point>);
        pcl::PointCloud<pcl_point>::Ptr keypoints2 (new pcl::PointCloud<pcl_point>);

        keypoints_search<pcl_point> ks;
        int n=4;
        ks.setMethod(atoi(argv[n]));
        if(atoi(argv[n])==1) //SIFT
        {
                ks.setParameters(atof(argv[n+1]), atoi(argv[n+2]), atoi(argv[n+3]));
                n=n+4;
        }
        else if(atoi(argv[n])==2)//ISS
        {
                float radius= atof(argv[n+3]);
                ks.setParameters(atof(argv[n+1]),atoi(argv[n+2]), radius);
                n=n+4;
        }

        ks.setTree(tree1);
        ks.setInputCloud(cloud1);
        ks.setInputNormals(normals1);
        ks.setCloudResolution(resolution1);
        ks.getKeypoints(keypoints1);

        std::cerr << "Number of points " << cloud1->width * cloud1->height << std::endl;
        std::cerr << "Number of keypoints " << keypoints1->width * keypoints1->height << std::endl;
        display_clouds(cloud1, keypoints1, color2, color1, 1, 5);

        ks.setTree(tree2);
        ks.setInputCloud(cloud2);
        ks.setInputNormals(normals2);
        ks.setCloudResolution(resolution2);
        ks.getKeypoints(keypoints2);

        std::cerr << "Number of points " << cloud2->width * cloud2->height << " data points." << std::endl;
        std::cerr << "Number of keypoints " << keypoints2->width * keypoints2->height << " data points." << std::endl;
        display_clouds(cloud2, keypoints2, color2, color1, 1, 5);

        pcl::registration::CorrespondenceEstimation<pcl_point, pcl_point> est;
        est.setInputSource(keypoints1);
        est.setInputTarget(keypoints2);
        pcl::Correspondences correspondences;
        est.determineCorrespondences(correspondences);

        pcl::registration::CorrespondenceRejectorOneToOne rejector_oto;
        rejector_oto.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
        rejector_oto.getCorrespondences(correspondences);

        pcl::registration::CorrespondenceRejectorMedianDistance rejector_dist;
        rejector_dist.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
        rejector_dist.setMedianFactor(4.0); //Points with distance greater than median times factor will be rejected
        rejector_dist.getCorrespondences(correspondences);
        //display_correspondences(keypoints1, keypoints2, correspondences);
        display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, correspondences);
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

 //Get transformation and transform src to tgt

        //std::cout<<"Getting transformation"<<std::endl<<std::endl;
        transformation_estimator<pcl_point> te;
        te.setInputClouds(keypoints1, keypoints2);
        te.setInputCorrespondences(&correspondences);
        Eigen::Matrix4f* initial_transfo;
        te.getTransfo(initial_transfo);
        std::cout<<*initial_transfo<<std::endl<<std::endl;
        pcl::PointCloud<pcl_point>::Ptr pre_aligned (new pcl::PointCloud<pcl_point>);
        te.getTransformedPointcloud(cloud1, pre_aligned);
        display_clouds(pre_aligned, cloud2, color1, color2, 1, 1);

        return 0;
}
