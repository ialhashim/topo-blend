#pragma once
#include "RelationDetector.h"

class GroupRelationDetector :
	public RelationDetector
{
public:
	GroupRelationDetector(Structure::Graph* g, int ith, double normalizeCoef, int logLevel): RelationDetector(g, "PairRelationDetector-", ith, normalizeCoef, 1, logLevel)
	{
	}
    QVector<GroupRelation> groupRelations_;	

    void detect(QVector<PairRelation>& prRelations);
protected:
	void detectSymmGroupByTransPair(QVector<PairRelation>& prRelations);
	void detectSymmGroupByRefPair(QVector<PairRelation>& prRelations);
    bool isIntersected(QSet<QString>& ids, PairRelation& pr2);
    void removeRedundantGroup();
	// is ids2 is a subgroup of ids1.
    bool isSubgroup(QVector<QString>& ids1, QVector<QString>& ids2);
    QString symmTypeOfTansPair(PairRelation& pr1);

	void detectCoplanarGroup(QVector<PairRelation>& prRelations);
	// merge coplanar group
	void mergeCoplanarGroup();
};

