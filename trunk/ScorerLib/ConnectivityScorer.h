#pragma once
#include "RelationDetector.h"

class ConnectivityScorer :	public RelationDetector
{
public:
	ConnectivityScorer(Structure::Graph* g, int ith, double normalizeCoef, bool isUseLink = true, int logLevel=0):RelationDetector(g, "ConnectivityScorer-", ith, normalizeCoef, 1, logLevel)
    {
		isUseLink_ = isUseLink;
    }
	double evaluate(QVector<QVector<PairRelation> > &connectPairs, QVector<PART_LANDMARK> &corres);
private:
	Structure::Link* findLink(Structure::Node *n1, Structure::Node *n2);
	double computeDeviationByLink(Structure::Link* link);

	double computeDeviationByDistance(Eigen::MatrixXd& m1, Structure::Node *n2);

	bool isUseLink_;
};

