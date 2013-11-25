#pragma once
#include "relationdetector.h"
class PairRelationScorer :
	public RelationDetector
{
public:
	PairRelationScorer(Structure::Graph* g, int ith, int logLevel=0):RelationDetector(g, "PairScorer-", ith, 0, logLevel)
    {
    }
	double evaluate(QVector<QVector<PairRelation> > &pairss, QVector<PART_LANDMARK> &corres);
private:
	void isParalOrthoCoplanar(PairRelation &cpr);
};

