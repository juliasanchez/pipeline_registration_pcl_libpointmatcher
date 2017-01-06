#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include "boost/filesystem.hpp"
#include <fstream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints PointCloud;
typedef PM::Parameters Parameters;

int validateArgs(const int argc, const char *argv[],string& initTranslation, string& initRotation);
PM::TransformationParameters parseTranslation(string& translation);
PM::TransformationParameters parseRotation(string& rotation);
void usage(const char *argv[]);
void BoundingBox (PointCloud cloud, float* volume);

int main(int argc, const char *argv[])
{
    string initTranslation("0,0,0");
    string initRotation("1,0,0;0,1,0;0,0,1");
    const int ret = validateArgs(argc, argv, initTranslation, initRotation);
    setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
    const char *src_file="/home/julia/Documents/test_libpointmatcher/data_local_frame/Hokuyo_2.csv";
    PointCloud source(PointCloud::load(src_file));
    const char *tgt_file="/home/julia/Documents/test_libpointmatcher/data_local_frame/Hokuyo_3.csv";
    PointCloud target(PointCloud::load(tgt_file));

    std::stringstream sstm;
    int src;
    int tgt;

    for (int i =2; i<20; i++)
    {
      src=i;
      tgt=i+1;
  //---------------------------------------STEP 1 : FILTERS----------------------------------------------------------------------

      ifstream file("configuration/filters_configuration.yaml");
      PM::DataPointsFilters filter(file);
      filter.apply(source);
      filter.apply(target);

//      int nbPointsSource = source.getNbPoints();
//      int nbPointsTarget = target.getNbPoints();
      float volumeSource;
      float volumeTarget;
      BoundingBox(source, &volumeSource);
      BoundingBox(target, &volumeTarget);

      std::cout<<"volume source :"<<volumeSource<<std::endl<<"volume target :"<<volumeTarget<<std::endl<<std::endl;

      //------------------------------------
      if(volumeSource>volumeTarget)
        {
        PM::swapDataPoints(source,target);
        src=i+1;
        tgt=i;
        }
      //this part is used to minimize wrong correspondences (when the algorithm search to attribute matches that doesn't appear on the target pointcloud)
      //------------------------------------

  //---------------------------------------STEP 2 : INITIAL TRANSFORMATION-------------------------------------------------------

  //initialisation de la transformation
      PM::TransformationParameters translation1 = parseTranslation(initTranslation);
      Eigen::Matrix<float, 4, 4> translation=translation1;//problème avec la multiplication dynamique avec eigen: Je passe par un intermédiaire statique
      PM::TransformationParameters rotation1 = parseRotation(initRotation);
      Eigen::Matrix<float, 4, 4> rotation=rotation1;//problème avec la multiplication dynamique avec eigen: Je passe par un intermédiaire statique
      PM::TransformationParameters initTransfo = translation*rotation;

      PM::Transformation* rigidTrans;
      rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

      if (!rigidTrans->checkParameters(initTransfo))
      {
          cerr << endl<< "Initial transformation is not rigid, identiy will be used"<< endl;
          initTransfo.setIdentity(4,4);
      }

      const PointCloud initializedSource = rigidTrans->compute(source, initTransfo);

  //---------------------------------------STEP 3 : ICP--------------------------------------------------------------------------
      PM::ICP icp;

      //first icp registration with large parameters for matches filtering
      ifstream ifs("configuration/icp_configuration.yaml");
      icp.loadFromYaml(ifs);

      PM::TransformationParameters T=icp(source,target);

      PointCloud source_out(source);
      icp.transformations.apply(source_out, T);

      //second icp registration with accurate parameters for matches filtering
      ifstream ifs1("configuration/icp_configuration1.yaml");
      icp.loadFromYaml(ifs1);

      T=icp(source_out,target);
      icp.transformations.apply(source_out, T);

  //---------------------------------------STEP 4 : EVALUATION---------------------------------------------------------------------

      cout << endl << "--------EVALUATION----------" << endl;

      // EVALUATION paired point mean distance without outliers.

      // initiate the matching
      icp.matcher->init(target);

      // extract closest points
      PM::Matches matches;
      matches = icp.matcher->findClosests(source_out);

      // weight paired points
      const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(source_out, target, matches);

      // generate tuples of matched points and remove pairs with zero weight
      const PM::ErrorMinimizer::ErrorElements matchedPoints( source_out, target, outlierWeights, matches);

      // extract relevant information for convenience
      const int dim = 3;
      const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
      const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim); // [x1,x2,x3...;y1,y2,y3...;z1,z2,z3;...]
      const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);

      PM::Matrix correspondences(matchedRead.rows()+matchedRef.rows(), matchedRead.cols());
      correspondences << matchedRead,
                         matchedRef;

      sstm.str("");
      sstm<<"matches_";
      sstm <<"src"<<src<<"_tgt"<<tgt<<".csv";
      std::string file_name = sstm.str();
      ofstream matchesFile;

      matchesFile.open(file_name.c_str());
      matchesFile << correspondences.transpose() << endl;
      matchesFile.close();

      sstm.str("");
      sstm<<"cd ~/Documents/test_libpointmatcher/iterated_registration/build-iterated_registration-Basic-Default ; awk '{printf(\"%g,%g,%g,%g,%g,%g\\n\",$1,$2,$3,$4,$5,$6)}' matches_src";
      sstm <<src<<"_tgt"<<tgt<<".csv > matches"<<src<<tgt<<".csv ; "<<"sed -i '1s/^/x1,y1,z1,x2,y2,z2\\n/' matches"<<src<<tgt<<".csv";
      std::string command = sstm.str();
      const char *command_char(command.c_str());
      system(command_char);

      // compute mean distance
      const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // compute the norm for each column vector =euclidian distance
      const float meanDist = dist.sum()/nbMatchedPoints;
      cout << "mean distance without outliers: " << meanDist << " m" << endl;

      cout << "------------------" << endl << endl;

      //---------------------------------------STEP 5 : SAVE RESULTS---------------------------------------------------------------------
      sstm.str("");
      sstm <<src<<"_to_"<<tgt<<".csv";
      std::string output_name = sstm.str();
      const char *output_file(output_name.c_str());
      source_out.save(output_file);

      //---------------------------------------STEP 6 : CHANGE SOURCE AND TARGET---------------------------------------------------------------------

      sstm.str("");
      sstm<<"/home/julia/Documents/test_libpointmatcher/data_local_frame/Hokuyo_";
      sstm <<i+1<<".csv";
      std::string file_address = sstm.str();
      const char *src_file(file_address.c_str());
      // Load new scan
      source=PointCloud::load(src_file);

      sstm.str("");
      sstm<<"/home/julia/Documents/test_libpointmatcher/data_local_frame/Hokuyo_";
      sstm <<i+2<<".csv";
      file_address = sstm.str();
      const char *tgt_file(file_address.c_str());
      // Load new scan
      target=PointCloud::load(tgt_file);

    }
    return 0;
}


void BoundingBox (PointCloud cloud, float* volume)
{
     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> x=cloud.features.row(0);
     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> y=cloud.features.row(1);
     Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z=cloud.features.row(2);
     *volume=(x.maxCoeff()-x.minCoeff())*(y.maxCoeff()-y.minCoeff())*(z.maxCoeff()-z.minCoeff());
}

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[],
                 string& initTranslation, string& initRotation)
{
  if (argc>1)
  {
    for (int i = 1; i < argc; i += 2)
    {
        const string opt(argv[i]);
        if (i + 1 > argc)
        {
            cerr << "Missing value for option " << opt;
            usage(argv);
            exit(1);
        }
        else if (opt == "--initTranslation")
        {
            initTranslation = argv[i+1];
        }
        else if (opt == "--initRotation")
        {
            initRotation = argv[i+1];
        }
        else
        {
            cerr << "Unknown option " << opt << ", usage:";
            usage(argv);
            exit(1);
        }
     }
   }
}

//---------------------------------------------------------------------------------------------------

PM::TransformationParameters parseTranslation(string& translation)
{
    const int cloudDimension=3;
    PM::TransformationParameters parsedTranslation;
    parsedTranslation = PM::TransformationParameters::Identity(
                cloudDimension+1,cloudDimension+1);

    translation.erase(std::remove(translation.begin(), translation.end(), '['),
                      translation.end());
    translation.erase(std::remove(translation.begin(), translation.end(), ']'),
                      translation.end());
    std::replace( translation.begin(), translation.end(), ',', ' ');
    std::replace( translation.begin(), translation.end(), ';', ' ');

    float translationValues[3] = {0};
    stringstream translationStringStream(translation);
    for( int i = 0; i < cloudDimension; i++)
    {
        if(!(translationStringStream >> translationValues[i]))
        {
            cerr << "An error occured while trying to parse the initial "
                 << "translation." << endl
                 << "No initial translation will be used" << endl;
            return parsedTranslation;
        }
    }
    float extraOutput = 0;
    if((translationStringStream >> extraOutput))
  {
        cerr << "Wrong initial translation size" << endl
             << "No initial translation will be used" << endl;
        return parsedTranslation;
    }

    for( int i = 0; i < cloudDimension; i++)
    {
        parsedTranslation(i,cloudDimension) = translationValues[i];
    }

    return parsedTranslation;
}

//---------------------------------------------------------------------------------------------------


PM::TransformationParameters parseRotation(string &rotation)
{
    const int cloudDimension=3;
    PM::TransformationParameters parsedRotation;
    parsedRotation = PM::TransformationParameters::Identity(cloudDimension+1,cloudDimension+1);
    rotation.erase(std::remove(rotation.begin(), rotation.end(), '['),rotation.end());
    rotation.erase(std::remove(rotation.begin(), rotation.end(), ']'),rotation.end());
    std::replace( rotation.begin(), rotation.end(), ',', ' ');
    std::replace( rotation.begin(), rotation.end(), ';', ' ');

    float rotationMatrix[9] = {0};
    stringstream rotationStringStream(rotation);
    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        if(!(rotationStringStream >> rotationMatrix[i])) {
            cerr << "An error occured while trying to parse the initial "
                 << "rotation." << endl
                 << "No initial rotation will be used" << endl;
            return parsedRotation;
        }
    }
    float extraOutput = 0;
    if((rotationStringStream >> extraOutput)) {
        cerr << "Wrong initial rotation size" << endl
             << "No initial rotation will be used" << endl;
        return parsedRotation;
    }

    for( int i = 0; i < cloudDimension*cloudDimension; i++) {
        parsedRotation(i/cloudDimension,i%cloudDimension) = rotationMatrix[i];
    }

    return parsedRotation;
}

// Dump command-line help
void usage(const char *argv[])
{
    cerr << " usage : " << argv[0]<<" ";
    cerr << "--initTranslation [x,y,z]  Add an initial 3D translation before applying ICP (default: 0,0,0)" << endl;
    cerr << "--initTranslation [x,y]    Add an initial 2D translation before applying ICP (default: 0,0)" << endl;
    cerr << "--initRotation [r00,r01,r02,r10,r11,r12,r20,r21,r22]" << endl;
    cerr << endl;
}
