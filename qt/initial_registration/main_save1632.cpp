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
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <string>

// PCL INCLUDES

using pcl::transformPointCloud;

//typedef pcl::PointXYZRGB pcl_point;
typedef pcl::PointXYZI pcl_point;
typedef Eigen::Matrix4f Matrix;
//typedef pcl::FPFHSignature33 descriptor;
typedef pcl::UniqueShapeContext1960 descriptor;

void getFeature_3dsc(pcl::PointCloud<pcl_point>::Ptr cloudin,pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::ShapeContext1980>* features);
void getFeature_usc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::UniqueShapeContext1960>* features);
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

//    pcl::FPFHEstimation<pcl_point, pcl::Normal, descriptor> feature_estimation;
//    feature_estimation.setRadiusSearch(0.2f);
//    feature_estimation.setSearchSurface(cloud1);
//    feature_estimation.setInputCloud(keypoints1);
//    feature_estimation.setInputNormals(normals1);
//    feature_estimation.setSearchMethod(tree1);
//    feature_estimation.compute(features1);

//   pcl::FPFHEstimation<pcl_point, pcl::Normal, descriptor> feature_estimation2;
//    feature_estimation2.setRadiusSearch(0.2f);
//    feature_estimation2.setSearchSurface(cloud2);
//    feature_estimation2.setInputCloud(keypoints2);
//    feature_estimation2.setInputNormals(normals2);
//    feature_estimation2.setSearchMethod(tree2);
//    feature_estimation2.compute(features2);

//    getFeature_3dsc(cloud1, keypoints1, normals1, tree1, 0.2f, &features1);
//    getFeature_3dsc(cloud2, keypoints2, normals2, tree2, 0.2f, &features2);

    float descriptor_radius= atof(argv[n]);
    n++;

    getFeature_usc(cloud1, keypoints1, tree1, descriptor_radius, &features1);
    getFeature_usc(cloud2, keypoints2, tree2, descriptor_radius, &features2);


//    for (size_t i = 0; i < features1.size (); ++i)
//    {
//        std::cout<<"descriptor[100]: "<<features1.at(i).descriptor[100]<<std::endl<<std::endl;
//        std::cout<<"descriptor[100]: "<<features2.at(i).descriptor[100]<<std::endl<<std::endl;
//    }

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
    int n_corr=0;
    int tot_n_corr=0;

    float diff;
    float temp;
    int pos;

    for (size_t i = 0; i < features1.size (); ++i)
    {
       // if (!pcl_isfinite (features1.points[i].histogram[0])) //skipping NaNs
          if (!pcl_isfinite (features1.at(i).descriptor[0])) //skipping NaNs
          {
            ++keypoints_skipped;
            continue;
          }

////manual method to compute the correspondence of the points
//      for(size_t j = 0; j < features2.size (); ++j)
//      {
//          diff=0;
//          for(int k = 0; k < 1960; ++k)
//          {
//            diff=diff+(features2.at(j).descriptor[k]-features1.at(i).descriptor[k])*(features2.at(j).descriptor[k]-features1.at(i).descriptor[k]);
//          }
//          if(diff<temp || j==0)
//          {
//              temp=diff;
//              pos=static_cast<int> (j);
//          }
//      }

//      std::cout << "1_ minimum distance: " <<temp<<std::endl<<std::endl;
//      std::cout <<"1_ point in cloud2 for minimum distance: " << pos<<std::endl<<std::endl;


//        float mini = *std::min_element(std::begin(diff), std::end(diff));
//        std::cout << "1 minimum distance: " <<mini<<std::endl<<std::endl;
//        std::cout <<"1 point in cloud2 for minimum distance: " << std::distance(std::begin(diff), &mini)<<std::endl<<std::endl;


      int found_neighs = match_search.nearestKSearch (features1.at (i), 1, neigh_indices, neigh_sqr_dists);
      std::cout<<"2_ minimum distance:"<<neigh_sqr_dists[0]<<std::endl<<std::endl;
      std::cout << "2_ point in cloud2 for minimum distance: " << neigh_indices[0]<<std::endl<<std::endl;

      if(found_neighs == 1 && neigh_sqr_dists[0] < atof(argv[n])) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      {
        pcl::Correspondence corr (static_cast<int> (i), neigh_indices[0], neigh_sqr_dists[0]);
        correspondences->push_back (corr);
        correspondences_global->push_back (corr);
        n_corr++;
        tot_n_corr++;
      }
      else if (found_neighs == 1)
      {
        pcl::Correspondence corr (static_cast<int> (i), neigh_indices[0], neigh_sqr_dists[0]);
        correspondences_global->push_back (corr);
        tot_n_corr++;
      }
    }
    n++;

        std::cout<<"number of skipped keypoints : "<<keypoints_skipped<<std::endl<<std::endl;
        std::cout<<"total number of correspondences: "<<tot_n_corr<<std::endl<<std::endl;
        std::cout<<"number of correspondences selected : "<<n_corr<<std::endl<<std::endl;

      display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences);
      display_correspondences_with_clouds(cloud1, keypoints1, cloud2, keypoints2, *correspondences_global);

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

void getFeature_3dsc(pcl::PointCloud<pcl_point>::Ptr cloudin, pcl::PointCloud<pcl_point>::Ptr keypoints, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::search::KdTree<pcl_point>::Ptr tree, float descriptor_radius, pcl::PointCloud<pcl::ShapeContext1980>* features)
{
        pcl::ShapeContext3DEstimation<pcl_point, pcl::Normal, pcl::ShapeContext1980> feature_estimation;
        feature_estimation.setSearchMethod(tree);

        feature_estimation.setSearchSurface(cloudin);
        feature_estimation.setInputCloud(keypoints);
        feature_estimation.setInputNormals(normals);

        feature_estimation.setMinimalRadius(descriptor_radius / 10.0);
        feature_estimation.setRadiusSearch(descriptor_radius);
        feature_estimation.setPointDensityRadius(descriptor_radius/ 5.0);
        feature_estimation.compute(*features);
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
