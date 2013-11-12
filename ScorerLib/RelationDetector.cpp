#include "RelationDetector.h"

QTextStream& operator << (QTextStream& os, const PairRelationBasic& pr)
{    
	os << "Pair <" << pr.n1->id << ", " << pr.n2->id << "> size <" << pr.n1->bbox().diagonal().norm() << ", " << pr.n2->bbox().diagonal().norm() << 
		"> Skeleton distance: " << pr.miniDist << "\n";
    return os;
}


void ConnectedPairDetector::detect()
{
	pairs_.clear();

	double dist = graph_->bbox().diagonal().norm();
	int tmp1 = graph_->edges.size();
	
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link* link = graph_->edges[i];
		SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
		SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
		
		PairRelationBasic prb;
		prb.n1 = link->n1;
		prb.n2 = link->n2;
		prb.miniDist = (p1-p2).norm()/dist;
		pairs_.push_back(prb);

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