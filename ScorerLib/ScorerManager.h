#ifndef SCORERMANAGER_H
#define SCORERMANAGER_H

#include <QObject>
#include "Scorer.h"
#include "RelationDetector.h"

class GraphCorresponder;
class Scheduler;


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
	QVector<QVector<PairRelationBasic> > connectPairs_; // connectPairs[0] is from source shape, [1] is from target shape

	// for debuging
	int logLevel_;
signals:
    void message(QString);

public slots:
	void parseConstraintPair();
    void parseGlobalReflectionSymm();
    void evaluateGlobalReflectionSymm();
	void evaluateGlobalReflectionSymmAuto();
	QVector<double> evaluateGlobalReflectionSymm( QVector<Structure::Graph*> const &graphs );

	void setIsUseSourceCenter(bool);

	void init();
	void clear();

private:
	bool ScorerManager::isGlobalReflectionSymmParsed()
	{
		return (-1 != maxGlobalSymmScore_);
	}
};
	

#endif // SCORERMANAGER_H
