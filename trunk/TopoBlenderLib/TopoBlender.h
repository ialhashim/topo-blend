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
    explicit TopoBlender(Structure::Graph * graph1, Structure::Graph * graph2, Scheduler * useScheduler, QObject *parent = 0);
    
	Structure::Graph * sg;
	Structure::Graph * active;
    Structure::Graph * tg;

	GraphCorresponder * gcoor;

	Scheduler * scheduler;

    QMap<QString, QVariant> params;

    // Logging
    int stepCounter;

public slots:
    // Experiments
    void testScheduler();

    // Logging
    void visualizeActiveGraph(QString caption, QString subcaption);

    // Preprocessing
    void cleanup();

    // Blending
    Structure::Graph * blend();
    void materializeInBetween( Structure::Graph * graph, double t, Structure::Graph * sourceGraph );

public:
    // DEBUG:
    std::vector< Vector3 > debugPoints;
    std::vector< PairVector3 > debugLines;
    void drawDebug();

signals:

};
