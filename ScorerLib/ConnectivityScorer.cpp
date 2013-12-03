#include "ConnectivityScorer.h"

double ConnectivityScorer::evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres)
{	
	double resultMax(0.0); int num(0);//double resultMean(0.0);
	double dist = graph_->bbox().diagonal().norm();

	for ( int j = 0; j < connectPairs.size(); ++j)
	{
		QVector<PairRelation>& pairs = connectPairs[j];
		for ( int i = 0; i < pairs.size(); ++i)
		{
			PairRelation& prb = pairs[i];

			std::vector<Structure::Node*> nodes1 = findNodesInB(prb.n1->id, graph_, corres, j==0);
			std::vector<Structure::Node*> nodes2 = findNodesInB(prb.n2->id, graph_, corres, j==0);

			double min_dist, mean_dist, max_dist;
			for ( int i1 = 0; i1 < (int) nodes1.size(); ++i1)
			{
				Eigen::MatrixXd m1 = node2matrix(nodes1[i1], pointsLevel_);

				for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
				{
					Eigen::MatrixXd m2 = node2matrix(nodes2[i2], pointsLevel_);					
					distanceBetween(m1, m2, min_dist, mean_dist, max_dist);
					
					min_dist = min_dist/dist;
					if (min_dist < prb.deviation)
					{
						min_dist = 0;
					}
					else
					{
////////////////////////////////////					
						if (logLevel_>0 ) 
						{
							logStream_ << num << "\n";
							if ( j == 0 )
								logStream_ << "connected pairs in source shape \n";
							else
								logStream_ << "connected pairs in target shape \n";

							logStream_ << prb << "correspond to: \n";
							logStream_ << "<" << nodes1[i1]->id << ", " << nodes2[i2]->id << "> size <" 
								<< nodes1[i1]->bbox().diagonal().norm() << ", " << nodes1[i2]->bbox().diagonal().norm()
								<< ", "<< dist << ">:" << min_dist << ", before normalization is: " << min_dist*dist << "\n\n";
						}
////////////////////////////////////	
						min_dist = min_dist - prb.deviation;
					}

					//resultMean += min_dist;
					++num;
					if ( min_dist > resultMax)
					{
						resultMax = min_dist;
					}
				}
			}
		}
	}

	if (logLevel_>0 )
	{
		//logStream_ << "mean score: " << 1-resultMean/num << "\n";
		logStream_ << "max score: " << 1/(1+resultMax) << "\n";		
	}
	return 1/(1+resultMax);
}