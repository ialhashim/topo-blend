#pragma once

#include <QObject>

class GraphCorresponder;
class Scheduler;

#include "RelationDetector.h"

class RelationManager : public QObject
{
    Q_OBJECT
public:
    RelationManager(GraphCorresponder * graph_corresponder, Scheduler * scheduler, QVector<Structure::Graph*> input_graphs);

	GraphCorresponder * gcorr;
	Scheduler * scheduler;
	QVector<Structure::Graph*> inputGraphs;

	// Relations
    QVector< QVector<PairRelation> > prGroups; // each element is a list of pair constraint of a graph in tb->graphs
    QVector< QVector<GroupRelation> > grGroups;
    QVector< double> diagonals;

	// Parameters
    double globalSymmWeight;
    double pairWeight;
    double graphWeight;

	// Global scores
    double maxPairScore;
    double maxGroupScore;
    double maxGlobalSymmScore;

	// Options
	bool isTracePairs;
	bool isTraceGroups;
	bool isCheckGlobalSymm;

signals:
    void message(QString);

public slots:
    void parseGlobalReflectionSymm();//parseGlobalSymm
    void parseModelConstraintGroup();
    void parseModelConstraintPair();

    void traceModelConstraintsAuto();
    void traceModelConstraints();

	double computeScore(double groupScore, double pairScore);

    void setIsCheckGlobalSymm(bool);
    void setGlobalSymmWeight(const QString&);
    void setIsTracePairs(bool);
    void setPairWeight(const QString&);
    void setIsTraceGroups(bool);
    void setGraphWeight(const QString&);

	void init();
	void clear();
};
