//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "display_clouds.h"
#include "display_normals.h"
#include "display_correspondences.h"
#include "display_correspondences_with_clouds.h"
#include "keypoints_search.h"
#include "transformation_estimator.h"
#include "cloud.h"
//#include "transformation_estimator.h"
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/fpfh.h>
#include <string>

// PCL INCLUDES

using pcl::transformPointCloud;

//typedef pcl::PointXYZRGB pcl_point;
typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;
typedef pcl::FPFHSignature33 descriptor;


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

    int n=1;
        std::string pcd_file1( argv[n] );
        n++;
        std::string pcd_file2( argv[n] );
        n++;

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
        if(atoi(argv[n])==1)
        {
            cloud_src.sample();
        }
        pcl::search::KdTree<pcl_point>::Ptr tree1 (new pcl::search::KdTree<pcl_point>);
        cloud_src.getTree(tree1);
        cloud_src.getScale(&Vol1);
        double resolution1=cloud_src.computeCloudResolution();

        cloud_tgt.clean();
        if(atoi(argv[n])==1)
        {
            cloud_tgt.sample();
        }
        n++;
        typename pcl::search::KdTree<pcl_point>::Ptr tree2 (new pcl::search::KdTree<pcl_point>);
        cloud_tgt.getTree(tree2);
        cloud_tgt.getScale(&Vol2);
        double resolution2=cloud_tgt.computeCloudResolution();

        //display_clouds(cloud1, cloud2, color1, color2, 1, 1);

        std::cout<<"Volumes : "<<Vol1<<"   "<<Vol2<<std::endl<<std::endl;
        std::cout<<"resolutions : "<<resolution1<<"   "<<resolution2<<std::endl<<std::endl;

        std::cerr << "Number of points cloud1 : " << cloud1->size() << std::endl;
        std::cerr << "Number of points cloud2 : " << cloud2->size() << std::endl;

        //std::cout<<"Computing normals"<<std::endl<<std::endl;
        pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
        cloud_src.getNormals(0.15, normals1);
        cloud_tgt.getNormals(0.15, normals2);

        //display_normals(cloud1,normals1);
        //display_normals(cloud2,normals2);

 //searching keypoints

        //std::cout<<"Determining keypoints"<<std::endl<<std::endl;
        pcl::PointCloud<pcl_point>::Ptr keypoints1 (new pcl::PointCloud<pcl_point>);
        pcl::PointCloud<pcl_point>::Ptr keypoints2 (new pcl::PointCloud<pcl_point>);

        keypoints_search<pcl_point> ks;

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

        //display_clouds(cloud1, keypoints1, color2, color1, 1, 8);

        ks.setTree(tree2);
        ks.setInputCloud(cloud2);
        ks.setInputNormals(normals2);
        ks.setCloudResolution(resolution2);
        ks.getKeypoints(keypoints2);

        std::cerr << "Number of points " << cloud2->width * cloud2->height << " data points." << std::endl;
        std::cerr << "Number of keypoints " << keypoints2->width * keypoints2->height << " data points." << std::endl;
        //display_clouds(cloud2, keypoints2, color2, color1, 1, 8);

////Correspondences with distances

//        if(atoi(argv[8])==0)
//        {
//        pcl::registration::CorrespondenceEstimation<pcl_point, pcl_point> est;
//        est.setInputSource(keypoints1);
//        est.setInputTarget(keypoints2);
//        pcl::Correspondences correspondences;
//        est.determineCorrespondences(correspondences);

//        pcl::registration::CorrespondenceRejectorOneToOne rejector_oto;
//        rejector_oto.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
//        rejector_oto.getCorrespondences(correspondences);

//        pcl::registration::CorrespondenceRejectorMedianDistance rejector_dist;
//        rejector_dist.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences));
//        rejector_dist.setMedianFactor(10.0); //Points with distance greater than median times factor will be rejected
//        rejector_dist.getCorrespondences(correspondences);
//        display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, correspondences);

//        //Get transformation and transform src to tgt

//        //std::cout<<"Getting transformation"<<std::endl<<std::endl;
//        transformation_estimator<pcl_point> te;
//        te.setInputClouds(keypoints1, keypoints2);
//        te.setInputCorrespondences(&correspondences);
//        Eigen::Matrix4f* initial_transfo;
//        te.getTransfo(initial_transfo);
//        std::cout<<*initial_transfo<<std::endl<<std::endl;
//        pcl::PointCloud<pcl_point>::Ptr pre_aligned (new pcl::PointCloud<pcl_point>);
//        te.getTransformedPointcloud(cloud1, pre_aligned);
//        //display_clouds(cloud1, pre_aligned, color2, color1, 1, 1);
//        display_clouds(pre_aligned, cloud2, color1, color2, 1, 1);
//        }


 //Correspondences with descriptors

    pcl::FPFHEstimation<pcl_point, pcl::Normal, descriptor> feature_estimation;
    feature_estimation.setRadiusSearch(0.2f);
    feature_estimation.setSearchSurface(cloud1);
    feature_estimation.setInputCloud(keypoints1);
    feature_estimation.setInputNormals(normals1);
    feature_estimation.setSearchMethod(tree1);
    pcl::PointCloud<descriptor> features1;
    feature_estimation.compute(features1);

   pcl::FPFHEstimation<pcl_point, pcl::Normal, descriptor> feature_estimation2;
    feature_estimation2.setRadiusSearch(0.2f);
    feature_estimation2.setSearchSurface(cloud2);
    feature_estimation2.setInputCloud(keypoints2);
    feature_estimation2.setInputNormals(normals2);
    feature_estimation2.setSearchMethod(tree2);
    pcl::PointCloud<descriptor> features2;
    feature_estimation2.compute(features2);

    pcl::PointCloud<descriptor>::Ptr src_descriptor_ptr(new pcl::PointCloud<descriptor>);
    pcl::PointCloud<descriptor>::Ptr tgt_descriptor_ptr(new pcl::PointCloud<descriptor>);
    *src_descriptor_ptr=features1;
    *tgt_descriptor_ptr=features2;


    pcl::KdTreeFLANN<descriptor> match_search;
    match_search.setInputCloud (tgt_descriptor_ptr);


    //boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences());
    pcl::Correspondences* correspondences=new pcl::Correspondences();
    pcl::Correspondences* correspondences_global=new pcl::Correspondences();

    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    int keypoints_skipped =0;

    for (size_t i = 0; i < features1.size (); ++i)
    {
        if (!pcl_isfinite (features1.points[i].histogram[0])) //skipping NaNs
          {
            ++keypoints_skipped;
            continue;
          }
      int found_neighs = match_search.nearestKSearch (features1.at (i), 1, neigh_indices, neigh_sqr_dists);
     // std::cout<<"distance:"<<neigh_sqr_dists[0]<<std::endl<<std::endl;
      if(found_neighs == 1 && neigh_sqr_dists[0] < atof(argv[n])) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        correspondences->push_back (corr);
        correspondences_global->push_back (corr);
      }
      else if (found_neighs == 1)
      {
          pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
          correspondences_global->push_back (corr);
      }
    }
    n++;

        std::cout<<"number of skipped keypoints : "<<keypoints_skipped<<std::endl<<std::endl;

      display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);
      display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences_global);

//    pcl::registration::CorrespondenceEstimation<descriptor, descriptor> est;

//    est.setInputSource(src_descriptor_ptr);
//    est.setInputTarget(tgt_descriptor_ptr);

//    boost::shared_ptr<pcl::Correspondences> correspondences;
//    est.determineCorrespondences(*correspondences);
//    display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);

 //Get transformation and transform src to tgt

//      pcl::Correspondences* corres;
//      *corres=correspondences.get();

    //std::cout<<"Getting transformation"<<std::endl<<std::endl;
    transformation_estimator<pcl_point> te;
    te.setInputClouds(keypoints1, keypoints2);
    te.setInputCorrespondences(correspondences);
    Eigen::Matrix4f* initial_transfo;
    te.getTransfo(initial_transfo);
    std::cout<<*initial_transfo<<std::endl<<std::endl;
    pcl::PointCloud<pcl_point>::Ptr pre_aligned (new pcl::PointCloud<pcl_point>);
    te.getTransformedPointcloud(cloud1, pre_aligned);
    //display_clouds(cloud1, pre_aligned, color2, color1, 1, 1);
    display_clouds(pre_aligned, cloud2, color1, color2, 1, 1);

    return 0;
}
