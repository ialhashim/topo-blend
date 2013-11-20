#pragma once
#include "RelationDetector.h"

class ConnectivityScorer :	public RelationDetector
{
public:
	ConnectivityScorer(Structure::Graph* g, int ith, int logLevel=0):pointsLevel_(1),RelationDetector(g, "ConnectivityScorer-", ith, logLevel)
    {
    }
	double evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres);

	
	int pointsLevel_; // 0 for main control points, 1 for all control points, 2 for all points.
};

