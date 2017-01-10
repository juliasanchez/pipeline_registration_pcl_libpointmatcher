void display_clouds (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2, int* color1, int* color2, int size1, int size2)
{
  pcl::visualization::PCLVisualizer viewer ("Pointcloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud1_color (cloud1, color1[1], color1[2], color1[3]);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud2_color (cloud2, color2[1], color2[2], color2[3]);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud1, cloud1_color, "cloud1");
  viewer.addPointCloud (cloud2, cloud2_color, "cloud2");
  viewer.addCoordinateSystem (1.0, "cloud1", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size1, "cloud1");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size2, "cloud2");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (10);
    //boost::this_thread::sleep(boost::posix_time::microseconds(10000));
  }
}
