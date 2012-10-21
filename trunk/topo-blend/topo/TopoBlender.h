#pragma once

#include <QObject>
#include "StructureGraph.h"

namespace Structure{

class TopoBlender : public QObject
{
    Q_OBJECT
public:
    explicit TopoBlender(Graph * graph1, Graph * graph2, QObject *parent = 0);
    
    Graph * g1;
    Graph * g2;

    Graph blend( Scalar t = 0.5 );

signals:
    
public slots:
    
};

}

