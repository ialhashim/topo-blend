#pragma once

#include <QObject>
#include "DynamicGraph.h"
#include "StructureGraph.h"
#include "GraphDistance.h"

struct Structure::Graph;

class TopoBlender : public QObject
{
    Q_OBJECT
public:
    explicit TopoBlender(Structure::Graph * graph1, Structure::Graph * graph2, QObject *parent = 0);

    Structure::Graph * g1;
    Structure::Graph * g2;

    GraphDistance * originalGraphDistance;

    DynamicGraph source;
    DynamicGraph active;
    DynamicGraph target;

    QMap<QString, QVariant> params;

    QList< Structure::ScalarLinksPair > badCorrespondence( QString activeNodeID, QString targetNodeID,
        QMap< Structure::Link*, std::vector<Vec4d> > & coord_active, QMap< Structure::Link*, std::vector<Vec4d> > & coord_target );

    // Logging
    int stepCounter;

public slots:
    // Experiments
    void bestPartialCorrespondence();
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
