//ROUTINE TO REGISTER 2 POINTCLOUDS

#include "display_clouds.h"
#include "display_hist.h"
#include "get_cost_matrix.h"
#include "display_normals.h"
#include "display_correspondences.h"
#include "display_correspondences_with_clouds.h"
#include "keypoints_search.h"
#include "transformation_estimator.h"
#include "cloud.h"

#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <string>
#include <dlib/optimization/max_cost_assignment.h>
#include<pcl/visualization/histogram_visualizer.h>

// PCL INCLUDES

using pcl::transformPointCloud;

//typedef pcl::PointXYZRGB pcl_point;
typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;
typedef pcl::FPFHSignature33 descriptor;
//typedef pcl::UniqueShapeContext1960 descriptor;

void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features);
void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree,float descriptor_radius,    pcl::PointCloud<pcl::FPFHSignature33>* features);

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

        ks.setTree(tree2);
        ks.setInputCloud(cloud2);
        ks.setInputNormals(normals2);
        ks.setCloudResolution(resolution2);
        ks.getKeypoints(keypoints2);

        std::cerr << "Number of points " << cloud2->width * cloud2->height << " data points." << std::endl;
        std::cerr << "Number of keypoints " << keypoints2->width * keypoints2->height << " data points." << std::endl;

        int display_keypoints = atoi (argv[n]);
        n++;

        if (display_keypoints)
        {
        display_clouds(cloud1, keypoints1, color2, color1, 1, 8);
        display_clouds(cloud2, keypoints2, color2, color1, 1, 8);
        }

        pcl::PointCloud<descriptor> features1;
        pcl::PointCloud<descriptor> features2;

 //Correspondences with descriptors

    float descriptor_radius= atof(argv[n]);
    n++;

//    getFeature_usc(cloud1, keypoints1, tree1, descriptor_radius, &features1);
//    getFeature_usc(cloud2, keypoints2, tree2, descriptor_radius, &features2);

    getFeature_fpfh(cloud1,keypoints1, normals1, tree1,descriptor_radius,&features1);
    getFeature_fpfh(cloud2,keypoints2, normals2, tree2,descriptor_radius,&features2);

//    pcl::PointCloud<pcl_point>::Ptr keypoints (new pcl::PointCloud<pcl_point>);
//    keypoints->width    = 1;
//    keypoints->height   = 1;
//    keypoints->points.resize (keypoints->width * keypoints->height);

//    int selected_keypoint = atoi(argv[n]);
//    n++;
//    pcl::PointCloud<descriptor> features;
//    keypoints->points[0]=keypoints1->points[selected_keypoint];
//    getFeature_fpfh(cloud1,keypoints, normals1, tree1,descriptor_radius,&features);
//    display_clouds(cloud1, keypoints, color2, color1, 1, 8);
//    display_hist(&features);

    int size_cost=std::max(features1.size(),features2.size());
    dlib::matrix<float>* cost(size_cost,size_cost);
    get_cost_matrix(features1, features2, *cost);


    /// to transform into function

//    pcl::PointCloud<descriptor>::Ptr src_descriptor_ptr(new pcl::PointCloud<descriptor>);
//    pcl::PointCloud<descriptor>::Ptr tgt_descriptor_ptr(new pcl::PointCloud<descriptor>);
//    *src_descriptor_ptr=features1;
//    *tgt_descriptor_ptr=features2;

//    pcl::Correspondences* correspondences=new pcl::Correspondences();

//    int keypoints_skipped =0;
//    int n_corr=0;
//    int tot_n_corr=0;
//    int size_cost=std::max(features1.size(),features2.size());
//    dlib::matrix<int> cost(size_cost,size_cost);
//    cost=dlib::zeros_matrix<int> (size_cost,size_cost);

//    int inf = std::numeric_limits<float>::infinity();

//    for (size_t i = 0; i < size_cost; ++i)
//    {
//        if(i<features1.size())
//        {
//            if (!pcl_isfinite (features1.points[i].histogram[0])) //skipping NaNs
//           //   if (!pcl_isfinite (features1.at(i).descriptor[0])) //skipping NaNs
//              {
//                ++keypoints_skipped;
//                continue;
//              }

//            for(size_t j = 0; j < size_cost; ++j)
//            {
//                if(j<features2.size())
//                {
//                    for(int k = 0; k < 33; ++k)
////                  for(int k = 0; k < 1960; ++k)
//                    {
////                    cost(i,j)=cost(i,j)+(features2.at(j).descriptor[k]-features1.at(i).descriptor[k])*(features2.at(j).descriptor[k]-features1.at(i).descriptor[k]);
//                      cost(i,j)=cost(i,j)+(features2.at(j).histogram[k]-features1.at(i).histogram[k])*(features2.at(j).histogram[k]-features1.at(i).histogram[k]);

//                    }
//                }

//                // to be sure to assign all values I add a column with very little values, the row that is too heavy to assign to a column will be assigned to this new column.

//                else
//                {
//                   cost(i,j)=inf;
//                }
//            }
//        }

//        /// the part below is implemented implicitly in dlib
//        // to be sure to assign all values I add a row with very little values, the column that is too heavy to assign to a row will be assigned to this new column.

//        else
//        {
//            for(size_t j = 0; j < size_cost; ++j)
//            {
//                cost(i,j)=inf;
//            }
//        }
//    }
//        cost=-1.0*cost;

    ///
        std::cout<<"cost matrix : "<<std::endl<<*cost<<std::endl<<std::endl;
        std::vector<long> assignment = max_cost_assignment(cost);


        pcl::Correspondences* correspondences=new pcl::Correspondences();
        int n_corr=0;
        int tot_n_corr=0;
        std::cout<<" correspondences: ";
        for (int i=0; i<assignment.size(); ++i)
        {
//            std::cout<<"cost : "<<std::endl<<cost(i, assignment[i])<<std::endl<<std::endl;
//            std::cout<<"compared to : "<<std::endl<<-1.0*atof(argv[n])<<std::endl<<std::endl;
            if(cost(i, assignment[i]) > -1*atof(argv[n])) //  add match only if the squared descriptor distance is less than 0.25
            {
            pcl::Correspondence corr (static_cast<int> (i), assignment[i], cost(i, assignment[i]));
            correspondences->push_back (corr);
            n_corr++;
            tot_n_corr++;
            std::cout<<", "<<assignment[i];
            }
        }
        n++;
        std::cout<<std::endl<<std::endl;

        std::cout<<"number of correspondences: "<<tot_n_corr<<std::endl<<std::endl;

      display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);

//    pcl::registration::CorrespondenceEstimation<descriptor, descriptor> est;

//    est.setInputSource(src_descriptor_ptr);
//    est.setInputTarget(tgt_descriptor_ptr);

//    pcl::Correspondences* correspondences=new pcl::Correspondences();
//    est.determineCorrespondences(*correspondences);
//    display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);

 //Get transformation and transform src to tgt

    //std::cout<<"Getting transformation"<<std::endl<<std::endl;


 if (correspondences->size()>0)
 {
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
 }
 else
 {
    std::cout<<"not enough correspondences found to get a transformation"<<std::endl<<std::endl;
 }

    return 0;
}



void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features)
{
        pcl::UniqueShapeContext<pcl_point, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> feature_estimation;
        feature_estimation.setSearchMethod(tree);
        feature_estimation.setRadiusSearch(descriptor_radius); //5 cm recherche des voisins pour faire le descriptor
        feature_estimation.setInputCloud(keypoints);
        feature_estimation.setSearchSurface(cloudin);
        feature_estimation.setRadiusSearch(descriptor_radius); // neigbours to take into account
        feature_estimation.setMinimalRadius(descriptor_radius / 10.0); //to avoid being too sensitive in bins close to center
        feature_estimation.setPointDensityRadius(descriptor_radius / 5.0); //radius for local density
        std::cout<<"bins 1  :"<<feature_estimation.getAzimuthBins()<<std::endl;
        std::cout<<"bins 2  :"<<feature_estimation.getElevationBins()<<std::endl;
        std::cout<<"bins 3  :"<<feature_estimation.getRadiusBins()<<std::endl;

        feature_estimation.setLocalRadius(descriptor_radius); //radius for local frame
        feature_estimation.compute(*features);
}

void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree,float descriptor_radius,    pcl::PointCloud<pcl::FPFHSignature33>* features)
{
        pcl::FPFHEstimation<pcl_point, pcl::Normal, pcl::FPFHSignature33> feature_estimation;

        feature_estimation.setSearchMethod(tree);
        feature_estimation.setRadiusSearch(descriptor_radius); //10 cm recherche des voisins pour faire le descriptor
        feature_estimation.setSearchSurface(cloudin);
        feature_estimation.setInputCloud(keypoints);
        feature_estimation.setInputNormals(normals);
        feature_estimation.compute(*features);
}
