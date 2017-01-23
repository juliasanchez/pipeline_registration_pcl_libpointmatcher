//ROUTINE TO PRE-REGISTER 2 POINTCLOUDS WITH KEYPOINTS/DESCRIPTORS METHOD

//MY_INCLUDES
#include "display_clouds.h"
#include "display_hist.h"
#include "get_cost_matrix_fpfh.h"
#include "get_cost_matrix_usc.h"
#include "normalize_hist.h"
#include "display_normals.h"
#include "display_correspondences.h"
#include "display_correspondences_with_clouds.h"
#include "keypoints_search.h"
#include "transformation_estimator.h"
#include "cloud.h"

// PCL INCLUDES
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
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;

void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features);
void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree,float descriptor_radius,    pcl::PointCloud<pcl::FPFHSignature33>* features);

//...............................................................................................................................

int main(int argc, char** argv)
{

    if ( argc < 12)
    {
        std::cout << "NOT ENOUGH ARGUMENTS"<< std::endl<< std::endl;
        std::cout << "Usage: ./initial_registration filename1.pcd filename2.pcd  sample_files{0,1} keypoints_method lambda_ratio minimum_neighbors_number radius_keypoints display_keypoints?{0,1} descriptor_radius match_limit ransac_matches_rejection_threshold "
                            << std::endl
                            << std::endl;
        exit(EXIT_FAILURE);
    }

    if ( argc > 12 )
    {
        std::cout << "TOO MANY ARGUMENTS"<< std::endl<< std::endl;
        exit(EXIT_FAILURE);
    }


    ///LOAD FILES AND PREPROCESS THEM------------------------------------------------------------------------------

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

    ///SEARCHING KEYPOINTS------------------------------------------------------------------------------

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

    ///COMPUTE DESCRIPTORS OF KEYPOINTS------------------------------------------------------------------------------

    pcl::PointCloud<pcl::FPFHSignature33> features1_fpfh;
    pcl::PointCloud<pcl::FPFHSignature33> features2_fpfh;
    pcl::PointCloud<pcl::UniqueShapeContext1960> features1_usc;
    pcl::PointCloud<pcl::UniqueShapeContext1960> features2_usc;

    float descriptor_radius= atof(argv[n]);
    n++;

    getFeature_usc(cloud1, keypoints1, tree1, descriptor_radius, &features1_usc);
    getFeature_usc(cloud2, keypoints2, tree2, descriptor_radius, &features2_usc);

    getFeature_fpfh(cloud1,keypoints1, normals1, tree1,descriptor_radius,&features1_fpfh);
    getFeature_fpfh(cloud2,keypoints2, normals2, tree2,descriptor_radius,&features2_fpfh);

    //compute cost matrices and display them.............................................................................

    int size_cost=std::max(features1_fpfh.size(),features2_fpfh.size());
    dlib::matrix<int> cost_fpfh(size_cost,size_cost);
    get_cost_matrix_fpfh(features1_fpfh, features2_fpfh, &cost_fpfh);
    std::cout<<"cost matrix fpfh: "<<std::endl<<cost_fpfh<<std::endl<<std::endl;

    size_cost=std::max(features1_usc.size(),features2_usc.size());
    dlib::matrix<int> cost_usc(size_cost,size_cost);
    get_cost_matrix_usc(features1_usc, features2_usc, &cost_usc);
    std::cout<<"cost matrix usc: "<<std::endl<<cost_usc<<std::endl<<std::endl;

    dlib::matrix<int> total_cost(size_cost,size_cost);
    total_cost=cost_fpfh+cost_usc;
    std::cout<<"total cost matrix : "<<std::endl<<total_cost<<std::endl<<std::endl;
    std::vector<long> assignment = max_cost_assignment(total_cost);

    //assignment.........................................................................................................

    pcl::Correspondences* correspondences=new pcl::Correspondences();
    int n_corr=0;
    int tot_n_corr=0;
    std::cout<<" correspondences: ";
    for (int i=0; i<assignment.size(); ++i)
    {
        if(total_cost(i, assignment[i]) > -1*atof(argv[n])) //  add match only if the squared descriptor distance is less than...
        {
            pcl::Correspondence corr (static_cast<int> (i), assignment[i], total_cost(i, assignment[i]));
            correspondences->push_back (corr);
            n_corr++;
            tot_n_corr++;
            std::cout<<", "<<assignment[i];
        }
    }
    n++;
    std::cout<<std::endl<<std::endl;

    std::cout<<"number of correspondences: "<<tot_n_corr<<std::endl<<std::endl;

    //display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);

    ///FILTERING CORRESPONDENCES--------------------------------------------------------------------------------------------------------

    pcl::Correspondences correspondences_out=*correspondences;
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl_point> rejector;
    rejector.setInputSource(keypoints1);
    rejector.setInputTarget(keypoints2);
    rejector.setInputCorrespondences(boost::make_shared<const pcl::Correspondences>(correspondences_out));
    rejector.setInlierThreshold(atof(argv[n]));
    n++;
    rejector.setMaximumIterations(500);
    rejector.getCorrespondences(correspondences_out);
    display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, correspondences_out);
    
    ///GET TRANSFORMATION----------------------------------------------------------------------------------------------------------------

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

//_________________________________________________________________________________________________________________________________________________


void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features)
{
    pcl::UniqueShapeContext<pcl_point, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> feature_estimation;
    feature_estimation.setSearchMethod(tree);
    feature_estimation.setRadiusSearch(descriptor_radius);
    feature_estimation.setInputCloud(keypoints);
    feature_estimation.setSearchSurface(cloudin);
    feature_estimation.setRadiusSearch(descriptor_radius); // neigbours to take into account
    feature_estimation.setMinimalRadius(descriptor_radius / 10.0); //to avoid being too sensitive in bins close to center
    feature_estimation.setPointDensityRadius(descriptor_radius / 5.0); //radius for local density
    //        std::cout<<"bins 1  :"<<feature_estimation.getAzimuthBins()<<std::endl;
    //        std::cout<<"bins 2  :"<<feature_estimation.getElevationBins()<<std::endl;
    //        std::cout<<"bins 3  :"<<feature_estimation.getRadiusBins()<<std::endl;

    feature_estimation.setLocalRadius(descriptor_radius); //radius for local frame
    feature_estimation.compute(*features);
}

void getFeature_fpfh(   pcl::PointCloud<pcl_point>::Ptr cloudin,    pcl::PointCloud<pcl_point>::Ptr keypoints,    pcl::PointCloud<pcl::Normal>::Ptr normals,   pcl::search::KdTree<pcl_point>::Ptr tree,float descriptor_radius,    pcl::PointCloud<pcl::FPFHSignature33>* features)
{
    pcl::FPFHEstimation<pcl_point, pcl::Normal, pcl::FPFHSignature33> feature_estimation;

    feature_estimation.setSearchMethod(tree);
    feature_estimation.setRadiusSearch(descriptor_radius); //neighbors in this radius searched to make descriptors
    feature_estimation.setSearchSurface(cloudin);
    feature_estimation.setInputCloud(keypoints);
    feature_estimation.setInputNormals(normals);
    feature_estimation.compute(*features);
}
