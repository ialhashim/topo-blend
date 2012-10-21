#include "TopoBlender.h"
using namespace Structure;

TopoBlender::TopoBlender(Graph *graph1, Graph *graph2, QObject *parent) : QObject(parent)
{
    g1 = graph1;
    g2 = graph2;
}

Graph TopoBlender::blend(Scalar t)
{
    Graph b;

	

    return b;
}
