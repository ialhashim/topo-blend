#pragma once
#include "DynamicGraphGlobal.h"
#include "StructureGraph.h"

class DynamicGraph
{
public:
    DynamicGraph(Structure::Graph * fromGraph = 0);
	DynamicGraph clone();

	// ADD
    int addNode(Properties properties = noProperties(), int index = -1);
	void addEdge(int fromNode, int toNode);

	void cloneNode(int index);

	// GET
    int nodeIndex(QString property_name, QString property_value);
	QSet<int> adjNodes(int index);
	QString nodeType(int index);

	// REMOVE
	void removeNode( int idx );	
	void removeEdge(int fromNode, int toNode);

	int numNodes(){ return nodes.size(); }
	int numEdges(EdgeType t);
	int numSheets();
	int numCurves();

	// State
	GraphState State();
	void printState();
	void printNodeInfo(int index);
	GraphState difference(GraphState & other);
	operator GraphState() { return State(); }

	// Properties
	Structure::Graph * mGraph;
	int uniqueID;

	QMap<int, SimpleNode> nodes;
	QMap<int, SimpleEdge> edges;
	QMap<int, QSet<SimpleEdge> > adjacency;
};
