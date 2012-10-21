#include "DynamicGraph.h"

DynamicGraph::DynamicGraph(Structure::Graph *fromGraph)
{
    this->mGraph = fromGraph;
	this->uniqueID = 0;

	if(mGraph == NULL) return;
	
    // Add nodes
    foreach(Structure::Node * n, mGraph->nodes)
        addNode( SingleProperty("original", n->id) );

    // Add edges
    foreach(Structure::Link e, mGraph->edges)
        addEdge( nodeIndex("original", e.n1->id), nodeIndex("original", e.n2->id) );
}

DynamicGraph DynamicGraph::clone()
{
	DynamicGraph g;
	g.mGraph	= this->mGraph;
	g.nodes		= this->nodes;
	g.edges		= this->edges;
	g.adjacency = this->adjacency;
	g.uniqueID	= this->uniqueID;
	return g;
}

int DynamicGraph::addNode(Properties properties, int index)
{
    if(index < 0) index = nodes.size();
    nodes[index] = SimpleNode(index);
    nodes[index].property = properties;

	// Always increasing global identifier
	uniqueID++;

	return index;
}

void DynamicGraph::addEdge(int fromNode, int toNode)
{
    if(fromNode == toNode) assert(0);

    SimpleEdge e(fromNode, toNode);

    edges[edges.size()] = e;

    adjacency[fromNode].insert(e);
    adjacency[toNode].insert(e);
}

int DynamicGraph::nodeIndex(QString property_name, QString property_value)
{
    foreach(SimpleNode n, nodes)
        if(n.property.contains(property_name) && n.property[property_name] == property_value)
            return n.idx;
    return -1;
}

QString DynamicGraph::nodeType( int index )
{
	return mGraph->getNode(nodes[index].property["original"])->type();
}

void DynamicGraph::removeNode( int idx )
{
	// Remove edges
	foreach(int adj, adjNodes(idx))
		removeEdge(idx, adj);

	// Remove node and adjacency list
	nodes.remove(idx);
	adjacency.remove(idx);
}

void DynamicGraph::removeEdge(int fromNode, int toNode)
{
	if(fromNode == toNode) assert(0);

	SimpleEdge e(fromNode, toNode);
	
	QMapIterator<int, SimpleEdge> i(edges);
	while (i.hasNext()) {
		i.next();
		if(i.value() == e){
			edges.remove(i.key());
			break;
		}
	}

	adjacency[fromNode].remove(e);
	adjacency[toNode].remove(e);
}

QSet<int> DynamicGraph::adjNodes(int index)
{
	QSet<int> adj;

	foreach(SimpleEdge e, adjacency[index])
		adj.insert( e.n[0] == index ? e.n[1] : e.n[0] );

	return adj;
}

GraphState DynamicGraph::State()
{
	GraphState state;

	state.numSheets = numSheets();
	state.numCurves = numCurves();

	state.numSheetEdges = numEdges(SAME_SHEET);
	state.numCurveEdges = numEdges(SAME_CURVE);
	state.numMixedEdges = numEdges(CURVE_SHEET);

	return state;
}

void DynamicGraph::printState()
{
	GraphState state = this->State();
	state.print();
}

int DynamicGraph::numEdges( EdgeType t )
{
	int count = 0;

	foreach(SimpleEdge e, edges){
		switch(t)
		{
		case ANY_EDGE	: count++; break;
		case CURVE_SHEET:if(nodeType(e.n[0]) != nodeType(e.n[1]))	count++; break;
		case SAME_CURVE	:if(nodeType(e.n[0]) == Structure::CURVE && nodeType(e.n[0]) == nodeType(e.n[1]))	count++; break;
		case SAME_SHEET	:if(nodeType(e.n[0]) == Structure::SHEET && nodeType(e.n[0]) == nodeType(e.n[1]))	count++; break;
		}
	}

	return count;
}

int DynamicGraph::numSheets()
{
	int count = 0;
	foreach(SimpleNode n, nodes) if(nodeType(n.idx) == Structure::SHEET) count++;
	return count;
}

int DynamicGraph::numCurves()
{
	int count = 0;
	foreach(SimpleNode n, nodes) if(nodeType(n.idx) == Structure::CURVE) count++;
	return count;
}

void DynamicGraph::cloneNode( int index )
{
	QSet<int> adj = adjNodes( index );

	int newIndex = addNode( nodes[index].property, uniqueID );

	foreach(int ni, adj)
		addEdge(newIndex, ni);
}

void DynamicGraph::printNodeInfo( int index )
{
	qDebug() << "\nNode:";
	qDebug() << " idx  : " << nodes[index].idx;
	qDebug() << " type : " << nodeType(index);

	// Adjacent info
	QStringList adjList; 
	foreach(int ni, adjNodes(index)) adjList << QString::number(ni);
	qDebug() << " adj  : " << adjList.join(", ");
}

GraphState DynamicGraph::difference( GraphState & other )
{
	GraphState diff;
	GraphState my = State();

	diff.numSheets = my.numSheets - other.numSheets;
	diff.numCurves = my.numCurves - other.numCurves;
	diff.numSheetEdges = my.numSheetEdges - other.numSheetEdges;
	diff.numCurveEdges = my.numCurveEdges - other.numCurveEdges;
	diff.numMixedEdges = my.numMixedEdges - other.numMixedEdges;

	return diff;
}
