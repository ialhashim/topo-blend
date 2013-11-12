#include "ConnectivityScorer.h"

double ConnectivityScorer::evaluate(QVector<QVector<PairRelationBasic> > &connectPairs, QVector<PART_LANDMARK> &corres)
{
	double result(0.0); int num(0);
	double dist = graph_->bbox().diagonal().norm();

	for ( int j = 0; j < connectPairs.size(); ++j)
	{
		QVector<PairRelationBasic>& pairs = connectPairs[j];
		//if (logLevel_>0)
		//{
		//	if ( j == 0 )
		//		logStream_ << "connected pairs in source shape \n";
		//	else
		//		logStream_ << "\n\n connected pairs in target shape \n";
		//}
		for ( int i = 0; i < pairs.size(); ++i)
		{
			PairRelationBasic& prb = pairs[i];

			std::vector<Structure::Node*> nodes1 = findNodes(prb.n1->id, graph_, corres, j==0);
			std::vector<Structure::Node*> nodes2 = findNodes(prb.n2->id, graph_, corres, j==0);
			//if (logLevel_>0)
			//{
			//	logStream_ << prb << "\n correspond to: \n";
			//}

			double min_dist, mean_dist, max_dist;
			for ( int i1 = 0; i1 < (int) nodes1.size(); ++i1)
			{
				Eigen::MatrixXd m1 = node2matrix(nodes1[i1], pointsLevel_);

				for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
				{
					Eigen::MatrixXd m2 = node2matrix(nodes2[i2], pointsLevel_);					
					distanceBetween(m1, m2, min_dist, mean_dist, max_dist);
					
					min_dist = min_dist/dist;
					if (min_dist < prb.miniDist)
						min_dist = 0;
					else
						min_dist = min_dist - prb.miniDist;

					result += min_dist;
					++num;
					
					if (logLevel_>0 && min_dist > 0)
					{
						logStream_ << num << "\n";
						if ( j == 0 )
							logStream_ << "connected pairs in source shape \n";
						else
							logStream_ << "connected pairs in target shape \n";

						logStream_ << prb << "correspond to: \n";
						logStream_ << "<" << nodes1[i1]->id << ", " << nodes2[i2]->id << ">: " << min_dist << "\n\n";
					}
					
				}
			}
		}
	}

	if (logLevel_>0 )
	{
		logStream_ << "mean score: " << 1-result/num << "\n";
	}
	return 1-result/num;
}