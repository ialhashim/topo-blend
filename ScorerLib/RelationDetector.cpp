#include "RelationDetector.h"

QTextStream& operator << (QTextStream& os, const PairRelationBasic& pr)
{    
	os << "\nPair <" << pr.n1->id << ", " << pr.n2->id << ">\n";	
	os << "Skeleton distance: " << pr.miniDist << "\n";
    return os;
}


void ConnectedPairDetector::detect()
{
	pairs_.clear();

	double dist = g_->bbox().diagonal().norm();
	int tmp1 = nodesPts_.size()-1;
	for ( int i = 0; i < tmp1; ++i)
	{
		Eigen::MatrixXd ptsi = nodesPts_[i];
		for ( int j = i+1; j < tmp1+1; ++j)
		{
			Eigen::MatrixXd ptsj = nodesPts_[j];
			double tmpd = distanceBetween(ptsi, ptsj)/dist;
			if (tmpd < thIntersectDist_)
			{
				PairRelationBasic prb;
				prb.n1 = g_->nodes[i];
				prb.n2 = g_->nodes[j];
				prb.miniDist = tmpd;
				pairs_.push_back(prb);
			}
		}
	}
}