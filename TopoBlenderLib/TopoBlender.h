#pragma once

#include <QObject>
#include "DynamicGraph.h"
#include "StructureGraph.h"
#include "GraphDistance.h"
#include "GraphCorresponder.h"

class Scheduler;

struct Structure::Graph;

class TopoBlender : public QObject
{
    Q_OBJECT
public:
    explicit TopoBlender(Structure::Graph * graph1, Structure::Graph * graph2, GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent = 0);
    
	Structure::Graph * sg;
    Structure::Graph * tg;
	GraphCorresponder * gcoor;
	Scheduler * scheduler;

	void oldSetup();

    QMap<QString, QVariant> params;

    // Logging
    int stepCounter;

	// Super graphs
	Structure::Graph * super_sg;
	Structure::Graph * active;
	Structure::Graph * super_tg;
	QMap<QString, QString> superNodeCorr;
	QMap<QString, QString> superEdgeCorr;
	void generateSuperGraphs();
	void correspondSuperNodes();
	void correspondSuperEdges();

	// Helper functions
	bool isExtraNode(Structure::Node *node);
	bool isExtraEdge(Structure::Link *link);
	void tagEdge(Structure::Link *link, QString tag);
	bool taggedEdge(Structure::Link *link, QString tag);
	QVector<QString> cloneGraphNode(Structure::Graph *g, QString nodeID, int N);

	// Tasks
	void generateTasks();

public slots:
	void executeBlend();

// DEBUG:
public:
    std::vector< Vector3 > debugPoints;
    std::vector< PairVector3 > debugLines;
    void drawDebug();

signals:

};
