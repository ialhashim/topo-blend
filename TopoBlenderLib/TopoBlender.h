#pragma once

#include <QObject>
#include <QWidget>

#include "DynamicGraph.h"
#include "StructureGraph.h"
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
    explicit TopoBlender(GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent = 0);
    ~TopoBlender();
    QWidget * parentWidget;

	Structure::Graph * sg;
    Structure::Graph * tg;
	GraphCorresponder * gcoor;
	Scheduler * scheduler;
    QMap<QString, QVariant> params;

	/// Super graphs:
	Structure::Graph * super_sg;
	Structure::Graph * active;
	Structure::Graph * super_tg;
	QMap<QString, QString> superNodeCorr;
	QMap<QString, QString> superEdgeCorr;

	/// Super graphs operations:
	void generateSuperGraphs();

	void correspondSuperNodes();
	void correspondSuperEdges();

	void equalizeSuperNodeResolutions();
	void equalizeSuperNodeTypes();
	bool convertSheetToCurve(QString sheetID, QString curveID, Structure::Graph* sheetG, Structure::Graph* curveG);

	/// Helper functions:
	QVector<QString> cloneGraphNode(Structure::Graph *g, QString nodeID, int N);
	Structure::Link * addMissingLink( Structure::Graph *g, Structure::Link * link );

	void correspondTwoEdges( Structure::Link *slink, Structure::Link *tlink, bool isFlip, Structure::Graph* source );

	void postprocessSuperEdges();

	// Edge correspondence cases
	void correspondTrivialEdges( Structure::Graph * source, Structure::Graph * target );
	void correspondSimilarType( Structure::Graph * source, Structure::Graph * target );
	void connectNullNodes( Structure::Graph * source, Structure::Graph * target );
	void correspondChangedEnds( Structure::Graph * source, Structure::Graph * target );
	void correspondRemainingOfNull( Structure::Graph * source, Structure::Graph * target );
	void connectFloatingRealNodes(Structure::Graph * source, Structure::Graph * target);
	void removeRedundantEdges( Structure::Graph * source );

	// Query
	bool isExtraNode( Structure::Node *node );

	// Edge lists filtering
	QVector<Structure::Link*> edgesContain(QVector<Structure::Link*> edges, QString property_name);
	QVector<Structure::Link*> edgesNotContain(QVector<Structure::Link*> edges, QString property_name);
	
	/// Tasks:
	void generateTasks();

	/// Create widgets and such
	void setupUI();

// DEBUG:
public:
	std::vector< Vector3 > debugPoints;
	std::vector< PairVector3 > debugLines;
	void drawDebug();
	void debugSuperGraphs(QString info);
};
