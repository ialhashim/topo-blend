#pragma once
#include "RelationDetector.h"

class GroupRelationDetector :
	public RelationDetector
{
public:
	GroupRelationDetector(Structure::Graph* g, int ith, double normalizeCoef, int logLevel): RelationDetector(g, "GroupRelationDetector-", ith, normalizeCoef, 1, logLevel)
	{
		if ( this->logLevel_ > 0)
		{
			logStream_ << "Threshold for axis group: " << thAxisGroup_ << "\n";
			logStream_ << "Threshold for ref group: " << thRefGroup_ << "\n";
			logStream_ << "Normalization coefficient: " << normalizeCoef_ << "\n";
		}
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

};

