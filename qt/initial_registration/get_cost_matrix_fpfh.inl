void get_cost_matrix_fpfh(pcl::PointCloud<pcl::FPFHSignature33> features1, pcl::PointCloud<pcl::FPFHSignature33> features2, dlib::matrix<int>* cost_matrix)
{
    int size_cost=std::max(features1.size(),features2.size());

    int keypoints_skipped =0;
    dlib::matrix<int> cost(size_cost,size_cost);
    cost=dlib::zeros_matrix<int> (size_cost,size_cost);

    int inf = std::numeric_limits<int>::max()/4;

    for (size_t i = 0; i < size_cost; ++i)
    {
        if(i<features1.size())
        {
            if (!pcl_isfinite (features1.points[i].histogram[0])) //skipping NaNs
              {
                ++keypoints_skipped;
                continue;
              }

            for(size_t j = 0; j < size_cost; ++j)
            {
                if(j<features2.size())
                {
                    for(int k = 0; k < 33; ++k)
                    {
                      cost(i,j)=cost(i,j)+(features2.at(j).histogram[k]-features1.at(i).histogram[k])*(features2.at(j).histogram[k]-features1.at(i).histogram[k]);
                    }
                }

                // to be sure to assign all values I add a column with very little values, the row that is too heavy to assign to a column will be assigned to this new column.

                else
                {
                   cost(i,j)=inf;
                }
            }
        }

        /// the part below is implemented implicitly in dlib
        // to be sure to assign all values I add a row with very little values, the column that is too heavy to assign to a row will be assigned to this new column.

        else
        {
            for(size_t j = 0; j < size_cost; ++j)
            {
                cost(i,j)=inf;
            }
        }
    }

    dlib::matrix<int> A(features1.size(),features2.size());
    A=dlib::subm(cost, dlib::range(0,features1.size()-1), dlib::range(0,features2.size()-1));
    A=A*1000/max(A);
    dlib::set_subm(cost,dlib::range(0,features1.size()-1), dlib::range(0,features2.size()-1) )=A;
        cost=-1.0*cost;
        *cost_matrix=cost;
}
