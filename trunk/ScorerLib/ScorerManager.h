#ifndef SCORERMANAGER_H
#define SCORERMANAGER_H

#include <QObject>
#include "Scorer.h"

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

	// Global scores
    double maxGlobalSymmScore;

	//
	Eigen::Vector3d refCenter_;// center of the reflection plane
	Eigen::Vector3d refNormal_; // normal of the reflection plane
	bool isUseSourceCenter_;

signals:
    void message(QString);

public slots:
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
		return -1 == maxGlobalSymmScore;
	}
};
	

#endif // SCORERMANAGER_H
