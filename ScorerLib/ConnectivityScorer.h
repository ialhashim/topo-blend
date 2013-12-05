#pragma once
#include "RelationDetector.h"

class ConnectivityScorer :	public RelationDetector
{
public:
	ConnectivityScorer(Structure::Graph* g, int ith, double normalizeCoef, int logLevel=0):RelationDetector(g, "ConnectivityScorer-", ith, normalizeCoef, 1, logLevel)
    {
    }
	double evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres);
};

