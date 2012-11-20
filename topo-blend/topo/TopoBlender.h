#pragma once

#include <QObject>
#include "DynamicGraph.h"

namespace Structure{

struct Graph;

class TopoBlender : public QObject
{
    Q_OBJECT
public:
    explicit TopoBlender(Graph * graph1, Graph * graph2, QObject *parent = 0);
    
    Graph * g1;
    Graph * g2;

	DynamicGraph source;
	DynamicGraph active;
	DynamicGraph target;

    Graph blend( Scalar t = 0.5 );
 
public slots:
	void bestPartialCorrespondence();

signals:

};

}
