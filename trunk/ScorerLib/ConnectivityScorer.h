#pragma once
#include "scorer.h"

class ConnectivityScorer :
	public Scorer
{
public:
	ConnectivityScorer(Structure::Graph* g, int ith, int logLevel=0):pointsLevel_(1),Scorer(g, "ConnectivityScorer-", ith, logLevel)
    {
    }
	double evaluate(QVector<QVector<PairRelationBasic> > &connectPairs, QVector<PART_LANDMARK> &corres);

	
	int pointsLevel_; // 0 for main control points, 1 for all control points, 2 for all points.
};

