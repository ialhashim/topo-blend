/*
	Ckmeans_1d_dp.cpp -- Performs 1-D k-means by a dynamic programming
	approach that is guaranteed to be optimal.

	Joe Song, Haizhou Wang
	Computer Science Department
	New Mexico State University
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <math.h>

using namespace std;

#define InputVector QVector<double>

namespace kmeans1D{

	/* data that return by kmeans.1d.dp()*/
	class data{
	public:
		vector<int> cluster;  	/*record which cluster each point belongs to*/
		vector<double> centers;	/*record the center of each cluster*/
		vector<double> withinss;/*within sum of distance square of each cluster*/
		vector<int> size;		/*size of each cluster*/
		int num_clusters;
	};

	/*one-dimensional cluster algorithm implemented in C*/
	/*x is input one-dimensional vector and K stands for the cluster level*/
	//all vectors in this program is considered starting at position 1, position 0 is not used.
	data kmeans( InputVector x, int K)
	{
		// Input:
		//  x -- a vector of numbers, not necessarily sorted
		//  K -- the number of clusters expected

		data result;
		int N = x.size()-1;  //N: is the size of input vector

		vector<double> temp(N);

		for(int i=0; i<N; i++)
			temp[i] = x[i+1];	//Initialize vector temp

		sort(temp.begin(), temp.end());
		temp.resize(unique( temp.begin(), temp.end())-temp.begin() );
		int vector_size = temp.size();

		if(vector_size < K)//The input array will be clustered to at most N clusters if K > N.
			K = vector_size;

		if(vector_size > 1) //The case when not all elements are equal.
		{ 
			vector<int> y(x.size());

			vector<double> temp_s = x.toStdVector();

			sort(temp_s.begin()+1, temp_s.end());

			for(int i=1; i<(int)x.size(); i++)
				for(int j=1; j<(int)x.size(); j++)
					if( x[i] == temp_s[j] )
					{	
						y[i] = j;
						break;
					}

					sort(x.begin()+1, x.end());
					vector< vector< double> > D( (K+1),vector<double>(N+1));
					vector< vector< double> > B( (K+1),vector<double>(N+1));

					int cubic = 0;    
					/*When cubic==1, which means "True", the algorithm runs in cubic time of array length N;
					otherwise it runs in quadratic time.  The TRUE option is for
					testing purpose only. */

					for(int i=1;i<=K;i++)
					{
						D[i][1] = 0;
						B[i][1] = 1;
					}

					double mean_x1, mean_xj,d;
					for(int k=1;k<=K;k++)
					{
						mean_x1 = x[1] ;

						for(int i=2;i<=N;i++)
						{
							if(k == 1) 
							{
								if(cubic) 
								{
									double sum=0, mean=0;
									for(int i=1;i<(int)x.size();i++)
										sum+=x[i];
									mean = sum/N;

									for(int i=1;i<(int)x.size();i++)
										D[1][i] += pow( (x[i] - mean), 2 );          

								} 
								else 
								{
									D[1][i] = D[1][i-1] + (i-1)/(double)i * pow( (x[i] - mean_x1),2 );
									mean_x1 = ((i-1) * mean_x1 + x[i])/(double)i;
								} 

								B[1][i] = 1;

							}
							else 
							{
								D[k][i] = -1;
								d = 0;
								mean_xj = 0;

								for(int j=i;j>=1;j--)
								{

									if(cubic) 
									{
										double sum=0, mean=0;
										for(int a=j;a<=i;a++)
											sum+=x[a];
										mean = sum/(i-j+1);

										for(int a=j;a<=i;a++)
											d += pow( (x[a] - mean), 2 );
									} 
									else 
									{

										d = d + (i-j)/(double)(i-j+1) * pow( (x[j] - mean_xj),2 );
										mean_xj = (x[j] + (i-j)*mean_xj)/(double)(i-j+1);
									}

									if(D[k][i] == -1) //initialization of D[k,i]
									{ 

										if(j == 1) 
										{        
											D[k][i] = d;
											B[k][i] = j;
										} 
										else 
										{ 
											D[k][i] = d + D[k-1][j-1];
											B[k][i] = j;
										}
									} 
									else 
									{
										if(j == 1) 
										{
											if(d <= D[k][i]) 
											{
												D[k][i] = d;
												B[k][i] = j;
											}
										} 
										else 
										{
											if(d + D[k-1][j-1] < D[k][i]) 
											{
												D[k][i] = d + D[k-1][j-1];
												B[k][i] = j;
											}
										}
									}
								}
							}
						}
					}

					//Backtrack to find the clusters of the data points
					int cluster_right = N;
					double cluster_left;
					result.cluster.resize(N+1);
					result.centers.resize(K+1);
					result.withinss.resize(K+1);
					result.size.resize(K+1);

					/*Forming final result*/
					for(int k=K;k>=1;k--)
					{
						cluster_left = B[k][cluster_right];

						for(int i=(int)cluster_left;i<=cluster_right;i++)
							result.cluster[i] = k;

						double sum=0;

						for(int a=(int)cluster_left;a<=cluster_right;a++)
							sum+=x[a];

						result.centers[k] = sum/(cluster_right-cluster_left+1);

						for(int a=(int)cluster_left;a<=cluster_right;a++)
							result.withinss[k] += pow( (x[a] - result.centers[k]), 2 );

						result.size[k] = cluster_right - (int)cluster_left + 1;

						if(k > 1) {
							cluster_right = (int)cluster_left - 1;
						}
					}

					//restore the original order
					vector<int> tt = result.cluster;
					for(int i=1; i<(int)x.size(); i++)
						result.cluster[i] = tt[ y[i] ];

		} else {  //a single cluster that contains all elements

			result.cluster.resize(N+1);
			result.centers.resize(2);
			result.withinss.resize(2);
			result.size.resize(2);

			for(int i=1;i<=N;i++)
				result.cluster[i] = N;

			result.centers[1] =x[1];
			result.withinss[1] = 0;
			result.size[1] = N;

		}

		result.num_clusters = result.size.size();

		return result;
	}
}
