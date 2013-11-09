#pragma once
#include "transform3d.h"

struct PairRelationBasic
{
	Structure::Node* n1;
	Structure::Node* n2;
	double miniDist;//mini distance between the skeleton' s control points of n1 & n2
    friend QTextStream& operator<<(QTextStream& os, const PairRelationBasic& pr);
};


class RelationDetector
{
public:
	RelationDetector(void)
	{
		thIntersectDist_ = 0.1;
	}

	double thIntersectDist_;
};

class ConnectedPairDetector : public RelationDetector
{
public:
	ConnectedPairDetector(Structure::Graph* g):g_(g)
	{
		std::vector<Eigen::Vector3d> cptsV;
		std::vector<int> numcpts;
		for ( int i = 0; i < (int) g_->nodes.size(); ++i)
		{
			Structure::Node * n = g_->nodes[i];

			std::vector<Eigen::Vector3d> nodeCptsV;
			int tmp = extractCpts( n, nodeCptsV, 2);

			Eigen::MatrixXd nodeCptsM;
			vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
			nodesPts_.push_back(nodeCptsM);		
		}
	}
	void detect();
	
	Structure::Graph* g_;
	QVector<Eigen::MatrixXd> nodesPts_;

	QVector<PairRelationBasic> pairs_;
};
