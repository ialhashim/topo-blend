#include "ConnectivityScorer.h"

Structure::Link* ConnectivityScorer::findLink(Structure::Node *n1, Structure::Node * n2)
{
	Structure::Link* link(0);
	int tmp1 = graph_->edges.size();
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link* tmpLink = this->graph_->edges[i];
		if (tmpLink->n1 == n1 && tmpLink->n2 == n2 || tmpLink->n1 == n2 && tmpLink->n2 == n1)
		{
			link = tmpLink;
			break;
		}		
	}
	return link;
}
double ConnectivityScorer::computeDeviationByLink(Structure::Link* link)
{
	double deviation(0.0);
	if (link)
	{
		SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
		SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
		deviation = (p1-p2).norm();
	}
	return deviation;
}
double ConnectivityScorer::computeDeviationByDistance(Eigen::MatrixXd& m1, Structure::Node *n2)
{
	double deviation, mean_dist, max_dist;
	Eigen::MatrixXd m2 = node2matrix(n2, this->pointLevel_);					
	distanceBetween(m1, m2, deviation, mean_dist, max_dist);
	return deviation;
}

double ConnectivityScorer::evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres)
{	
	double maxDeviation(0.0); int num(0);
	double deviation(0.0);

	for ( int j = 0; j < connectPairs.size(); ++j)
	{
		QVector<PairRelation>& pairs = connectPairs[j];
		for ( int i = 0; i < pairs.size(); ++i)
		{
			PairRelation& prb = pairs[i];

			std::vector<Structure::Node*> nodes1 = findNodesInB(prb.n1->id, graph_, corres, j==0);
			std::vector<Structure::Node*> nodes2 = findNodesInB(prb.n2->id, graph_, corres, j==0);

			
			for ( int i1 = 0; i1 < (int) nodes1.size(); ++i1)
			{
				Eigen::MatrixXd m1 = node2matrix(nodes1[i1], this->pointLevel_);

				for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
				{					
					Structure::Link* link = findLink(nodes1[i1], nodes2[i2]);	
					if ( this->isUseLink_)
						deviation = computeDeviationByLink(link);
					else
						deviation = computeDeviationByDistance(m1, nodes2[i2]);

					deviation = deviation/this->normalizeCoef_;

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
								<< nodes1[i1]->bbox().diagonal().norm() << ", " << nodes2[i2]->bbox().diagonal().norm()
								<< ">:" << deviation << "\n\n";
						}
				////////////////////////////////////	

					if (deviation <= prb.deviation)
					{
						deviation = 0;
					}
					else
					{
						deviation = deviation - prb.deviation;
					}
					++num;


					if ( deviation > maxDeviation)
					{
						maxDeviation = deviation;
					}
				}//for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
			}
		}
	}

	if (logLevel_>0 )
	{
		logStream_ << "max score: " << 1/(1+maxDeviation) << "\n";		
	}
	return 1/(1+maxDeviation);
}