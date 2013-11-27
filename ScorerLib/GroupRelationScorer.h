#pragma once
#include "relationdetector.h"
class GroupRelationScorer :
	public RelationDetector
{
public:
	GroupRelationScorer(Structure::Graph* g, int ith, int logLevel=0):RelationDetector(g, "GroupScorer-", ith, 1, logLevel){ }
	double evaluate(QVector<QVector<GroupRelation> > &groupss, QVector<PART_LANDMARK> &corres);
protected:
	GroupRelation findCorrespondenceGroup(Structure::Graph *graph, GroupRelation &gr,QVector<PART_LANDMARK>& corres,bool bSource);
	void computeGroupDeviationByCpts(GroupRelation& cgr, GroupRelation& gr);
	double errorOfCoplanarGroupByCpts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& point, Eigen::Vector3d& normal);
	void computeGroupAxis(GroupRelation& cgr, QString pairtype);
};

