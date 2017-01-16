void normalize_hist(pcl::PointCloud<pcl::FPFHSignature33>* features)
{
    int sum=0;

    for (size_t j = 0; j < features->size(); ++j)
    {
	sum=0;
	for(int k = 0; k < 33; ++k)
	{
		sum=sum+features->at(j).histogram[k];
	}
	
	for(int k = 0; k < 33; ++k)
	{
                features->at(j).histogram[k]=features->at(j).histogram[k]*100/sum;
	}
    }

}
