#pragma once

#include <QObject>

#include "DynamicGraph.h"
#include "StructureGraph.h"
#include "GraphDistance.h"
#include "GraphCorresponder.h"

class Scheduler;

struct SetNodes{
	QSet<Structure::Node*> set;
	QMap<QString,QVariant> property;
};

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

	/// Super graphs:
	Structure::Graph * super_sg;
	Structure::Graph * active;
	Structure::Graph * super_tg;
	QMap<QString, QString> superNodeCorr;
	QMap<QString, QString> superEdgeCorr;
	void generateSuperGraphs();
	void correspondSuperNodes();
	void equalizeSuperNodeResolutions();
	void equalizeSuperNodeTypes();
	bool convertSheetToCurve(QString sheetID, QString curveID, Structure::Graph* sheetG, Structure::Graph* curveG);
	void correspondSuperEdges();

	/// Helper functions:
	QVector<QString> cloneGraphNode(Structure::Graph *g, QString nodeID, int N);
	Structure::Link * addMissingLink( Structure::Graph *g, Structure::Link * link );
	Structure::Node * addMissingNode( Structure::Graph *toGraph, Structure::Graph * fromGraph, Structure::Node * fromNode );
	void tagEdge(Structure::Link *link, QString tag);
	bool taggedEdge(Structure::Link *link, QString tag);
	void correspondTwoEdges( Structure::Link *slink, Structure::Link *tlink, bool isFlip, Structure::Graph* source );
	QString correspondingNode( Structure::Link *link, int i );
	void removeMissingEdges( Structure::Graph * sgraph );
	void removeUncorrespondedEdges( Structure::Graph * graph );
	void remainingUncorrespondedEdges( Structure::Graph * source, Structure::Graph * target );
	void removeRedundantEdges( Structure::Graph * source );
	void postprocessSuperEdges();

	// Edge correspondence cases
	void correspondTrivialEdges( Structure::Graph * source, Structure::Graph * target );
	void correspondSimilarType( Structure::Graph * source, Structure::Graph * target );
	void connectNullNodes( Structure::Graph * source, Structure::Graph * target );
	void correspondChangedEnds( Structure::Graph * source, Structure::Graph * target );

	// Query
	bool isExtraNode(Structure::Node *node);
	bool isExtraEdge(Structure::Link *link);
	bool isCorrespondedEdge(Structure::Link *link);
	bool isShareCorrespondedNode( Structure::Link * slink, Structure::Link * tlink );

	// Edge lists filtering
	QVector<Structure::Link*> edgesContain(QVector<Structure::Link*> edges, QString property_name);
	QVector<Structure::Link*> edgesNotContain(QVector<Structure::Link*> edges, QString property_name);
	
	// Null sets
	QVector< SetNodes > nullNodeSets( Structure::Graph * sgraph, Structure::Graph * tgraph );
	QVector<Structure::Link*> nonCorrespondEdges( Structure::Node * node, Structure::Graph * graph );
	
	void checkIntermediateCuts(Structure::Graph * original, Structure::Graph * super_s, Structure::Graph * super_t);

	/// Tasks:
	void generateTasks();

	/// Logging
	int stepCounter;

public slots:
	void executeBlend();

// DEBUG:
public:
    std::vector< Vector3 > debugPoints;
    std::vector< PairVector3 > debugLines;
    void drawDebug();

signals:

};
