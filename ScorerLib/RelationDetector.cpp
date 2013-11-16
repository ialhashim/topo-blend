#include "RelationDetector.h"

Eigen::MatrixXd node2matrix(Structure::Node* node, int pointLevel)
{
	std::vector<Eigen::Vector3d> nodeCptsV;
	int tmp = extractCpts( node, nodeCptsV, pointLevel);
	Eigen::MatrixXd nodeCptsM;
	vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
	return nodeCptsM;
}

QTextStream& operator << (QTextStream& os, const PairRelationBasic& pr)
{    
	os << "Pair <" << pr.n1->id << ", " << pr.n2->id << "> size <" << pr.n1->bbox().diagonal().norm() << ", " << pr.n2->bbox().diagonal().norm() << 
		"> Skeleton distance: " << pr.miniDist << "\n";
    return os;
}
// bSource == true means that the id is from target shape, & we want to find its corresponded node in source shape, then graph should be source
std::vector<Structure::Node*> RelationDetector::findNodesInST(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource)
{	
	QVector<QString> ids;
    if ( bSource)
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids2 = corres[i].second;
			for ( int j = 0; j < ids2.size(); ++j)
            {
				if ( ids2[j] == id)
				{
					 ids = corres[i].first;
					 break;
				}
            }
			if ( !ids.isEmpty() )
				break;
		}
	}
	else
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids1 = corres[i].first;
			for ( int j = 0; j < ids1.size(); ++j)
            {
				if ( ids1[j] == id)
				{
					 ids = corres[i].second;
					 break;
				}
            }
			if ( !ids.isEmpty() )
				break;
		}
	}

	std::vector<Structure::Node*> result;
	for ( int i = 0; i < (int) ids.size(); ++i)
	{
		result.push_back( graph->getNode(ids[i]) );
	}
	return result;
}

void ConnectedPairDetector::detect(Structure::Graph* g, QVector<PART_LANDMARK> &corres)
{
	pairs_.clear();

	double dist = graph_->bbox().diagonal().norm();
	int tmp1 = graph_->edges.size();
	int num(0);
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link* link = graph_->edges[i];
		SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
		SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
		
		PairRelationBasic prb;
		prb.n1 = link->n1;
		prb.n2 = link->n2;
		prb.miniDist = (p1-p2).norm()/dist;

		
		std::vector<Structure::Node*> nodes1 = findNodesInST(prb.n1->id, g, corres, !bSource_);
		std::vector<Structure::Node*> nodes2 = findNodesInST(prb.n2->id, g, corres, !bSource_);
		double dist1 = g->bbox().diagonal().norm();
		double min_dist, mean_dist, max_dist,resultMax(0.0);		
		for ( int i1 = 0; i1 < (int) nodes1.size(); ++i1)
		{
			Eigen::MatrixXd m1 = node2matrix(nodes1[i1], pointsLevel_);

			for ( int i2 = 0; i2 < (int) nodes2.size(); ++i2)
			{
				Eigen::MatrixXd m2 = node2matrix(nodes2[i2], pointsLevel_);					
				distanceBetween(m1, m2, min_dist, mean_dist, max_dist);
					
				min_dist = min_dist/dist1;
				if ( min_dist > resultMax)
				{
					resultMax = min_dist;
				}					
					
				if (logLevel_>0 && min_dist > prb.miniDist ) //
				{
					logStream_ << num << "\n";
					if ( bSource_ )
					{
						logStream_ << "connected pair in source shape <" << prb.n1->id << ", " << prb.n2->id << "> with min dist: " << prb.miniDist << "\n";
						logStream_ << "corresponds to a pair in target shape <" << nodes1[i1]->id << ", " << nodes2[i2]->id << ">: " << min_dist << "\n\n";
					}
					else
					{
						logStream_ << "connected pairs in target shape <" << prb.n1->id << ", " << prb.n2->id << "> with min dist: " << prb.miniDist << "\n";
						logStream_ << "corresponds to a pair in source shape <" << nodes1[i1]->id << ", " << nodes2[i2]->id << ">: " << min_dist << "\n\n";
					}
					++num;
				}
					
			}
		}

		/////////
		//if (nodes1.size()*nodes2.size()>0)
		//{
			prb.miniDist = std::max(prb.miniDist, resultMax);
			pairs_.push_back(prb);
		//}
	}

	if( logLevel_ >1 )
	{
		logStream_ << "Total: " << pairs_.size() << " pairs" << "\n\n";
		for ( QVector<PairRelationBasic>::iterator it = pairs_.begin(); it != pairs_.end(); ++it)
			logStream_ << *it << "\n";
	}

	//int tmp1 = nodesPts_.size()-1;
	//for ( int i = 0; i < tmp1; ++i)
	//{
	//	Eigen::MatrixXd ptsi = nodesPts_[i];
	//	for ( int j = i+1; j < tmp1+1; ++j)
	//	{
	//		Eigen::MatrixXd ptsj = nodesPts_[j];
	//		double tmpd = distanceBetween(ptsi, ptsj)/dist;
	//		if (tmpd < thIntersectDist_)
	//		{
	//			PairRelationBasic prb;
	//			prb.n1 = graph_->nodes[i];
	//			prb.n2 = graph_->nodes[j];
	//			prb.miniDist = tmpd;
	//			pairs_.push_back(prb);
	//		}
	//	}
	//}
}