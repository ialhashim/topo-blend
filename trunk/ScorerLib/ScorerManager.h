#ifndef SCORERMANAGER_H
#define SCORERMANAGER_H

#include <QObject>
#include "RelationDetector.h"
#include "GroupRelationDetector.h"
class GraphCorresponder;
class Scheduler;


// todo jjcao trace pairs!
// todo jjcao parse & trance groups!

class ScorerManager : public QObject
{
	Q_OBJECT

public:
	ScorerManager(GraphCorresponder * graph_corresponder, Scheduler * scheduler, QVector<Structure::Graph*> input_graphs);

	GraphCorresponder * gcorr;
	Scheduler * scheduler;
	QVector<Structure::Graph*> inputGraphs;	

	// for global symm
    double maxGlobalSymmScore_;
	Eigen::Vector3d refCenter_;// center of the reflection plane
	Eigen::Vector3d refNormal_; // normal of the reflection plane
	bool isUseSourceCenter_;

	// for connectivity
	QVector<QVector<PairRelation> > connectPairs_; // connectPairs[0] is from source shape, [1] is from target shape

	QVector<QVector<PairRelation> > otherPairs_;
	QVector<QVector<GroupRelation> > groupRelations_;

	// for debuging
	int logLevel_;
signals:
    void message(QString);

public slots:
	//////////////
	void parseConstraintPair();
	void evaluateTopology();
	QVector<double> evaluateTopology( QVector<Structure::Graph*> const &graphs );
	void evaluateTopologyAuto();
	
	void parseConstraintGroup();
	void evaluateGroups();
	QVector<double> evaluateGroups( QVector<Structure::Graph*> const &graphs );
	void evaluateGroupsAuto();

	//////////////
    void parseGlobalReflectionSymm();
    void evaluateGlobalReflectionSymm();	
	QVector<double> evaluateGlobalReflectionSymm( QVector<Structure::Graph*> const &graphs );
	void evaluateGlobalReflectionSymmAuto();

	//void evaluatePairs();
	//QVector<double> evaluatePairs( QVector<Structure::Graph*> const &graphs );
	//void evaluatePairsAuto();

	void setIsUseSourceCenter(bool);

	//////////////
	void init();
	void clear();

private:
    bool isGlobalReflectionSymmParsed()
	{
		return (-1 != maxGlobalSymmScore_);
	}
	Structure::Graph * getCurrentGraph(int &idx);
};
	

#endif // SCORERMANAGER_H
