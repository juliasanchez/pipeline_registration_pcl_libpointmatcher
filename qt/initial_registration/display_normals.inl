void display_normals (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	pcl::visualization::PCLVisualizer viewer ("Pointcloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color (cloud, 0, 0, 255);
	// We add the point cloud to the viewer and pass the color handler
        viewer.addPointCloud (cloud, cloud_color, "cloud");
	viewer.initCameraParameters ();
	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, normals, 10, 0.08, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0, "normals");

	while (!viewer.wasStopped ()) 
	{ // Display the visualiser until 'q' key is pressed
	viewer.spinOnce (10);
	}
 }
