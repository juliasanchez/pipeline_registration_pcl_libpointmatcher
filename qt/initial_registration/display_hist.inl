void display_hist (pcl::PointCloud<pcl::FPFHSignature33 >* features)
{
    pcl::visualization::PCLHistogramVisualizer hist;
    hist.addFeatureHistogram(*features, 33 , "cloud", 640, 200);
    hist.spin();
}
