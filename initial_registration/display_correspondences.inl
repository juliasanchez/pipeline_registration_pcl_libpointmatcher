void display_correspondences (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, pcl::Correspondences correspondences)
{

pcl::visualization::PCLVisualizer viewer; 
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud1_color (cloud1, 255, 0, 0);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud2_color (cloud2, 0, 0, 255);

viewer.addPointCloud (cloud1, cloud1_color, "cloud1");
viewer.addPointCloud (cloud2, cloud2_color, "cloud2");

viewer.addCorrespondences<pcl::PointXYZI> (cloud1, cloud2, correspondences);

viewer.resetCamera (); 
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (10);
  } 
}
