#include "StructureGraph.h"
using namespace Structure;

Graph::~Graph()
{
    //qDeleteAll( nodes );
    //nodes.clear();
}

QBox3D Graph::bbox()
{
    return QBox3D();
}

Node *Graph::addNode(Node * n)
{
    Node * found = getNode( n->id );
    if(found) return found;

    nodes.insert(n);
    return n;
}

Link Graph::addEdge(Node *n1, Node *n2)
{
    if(n2->valence() > n1->valence())
        std::swap(n1, n2);

    n1 = addNode(n1);
    n2 = addNode(n2);

    Link e( n1, n2, "none", linkName(n1, n2) );
    edges.insert(e);

    return e;
}

QString Graph::linkName(Node *n1, Node *n2)
{
    return QString("%1 : %2").arg(n1->id).arg(n2->id);
}

Node *Graph::getNode(QString nodeID)
{
    foreach(Node* n, nodes)
        if(n->id == nodeID) return n;

    return NULL;
}

void Graph::draw() const
{
    foreach(Node * n, nodes)
    {
        n->draw();
    }

    foreach(Link e, edges)
    {
        e.draw();
    }
}

void Graph::draw2D() const
{

}
