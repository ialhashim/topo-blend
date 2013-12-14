#include "ConnectivityScorer.h"

double ConnectivityScorer::computeDeviationByDistance(Eigen::MatrixXd& m1, Structure::Node *n2)
{
	double deviation, mean_dist, max_dist;
	Eigen::MatrixXd m2 = node2matrix(n2, this->pointLevel_);					
	distanceBetween(m1, m2, deviation, mean_dist, max_dist);
	return deviation;
}

double ConnectivityScorer::evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres)
{	
	if (logLevel_>0 ) 
	{
		logStream_ << "normalize coeff is: " << this->normalizeCoef_ << "\n";
	}

	double maxDeviation(0.0); int num(0);
	double deviation(0.0);
	int k(0);
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
					Structure::Link* link = RelationDetector::findLink(nodes1[i1], nodes2[i2], this->graph_);	
					if ( this->isUseLink_)
						deviation = RelationDetector::computeDeviationByLink(link);
					else
						deviation = computeDeviationByDistance(m1, nodes2[i2]);

					deviation = deviation/this->normalizeCoef_;
					deviation = deviation - prb.deviation;
					++num;
					if ( deviation > maxDeviation)
					{
						maxDeviation = deviation;
						k = num;
					}

					if (logLevel_>0 ) 
					{
						logStream_ << num << "\n";
						if ( j == 0 )
							logStream_ << "connected pairs in source shape \n";
						else
							logStream_ << "connected pairs in target shape \n";

						logStream_ << prb << "correspond to: \n";
						logStream_ << "<" << nodes1[i1]->id << ", " << nodes2[i2]->id << ">\n";
						
						if ( link !=0)
						{
							SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
							SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
							logStream_ << "<" << p1.x() << ", " << p1.y() << ", " << p1.z() << "; " << p2.x() << ", " << p2.y() << ", " << p2.z() << ">\n";
						}
						if (link==0)
							this->logStream_ << "link does not exist, distance: ";
						else
							this->logStream_ << "link distance: "; 

						this->logStream_ << deviation + prb.deviation << " with deviation improved by:" << deviation << "\n\n";
					}
				}//for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
			}
		}
	}

	if (logLevel_>0 )
	{
		logStream_ << "pair: " << k << " with max deviation: "<< maxDeviation << " leads to score: " << 1/(1+maxDeviation) << "\n";		
	}
	return 1/(1+maxDeviation);
}