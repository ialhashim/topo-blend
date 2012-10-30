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

	int cloneNode(int index, bool cloneEdges = false);

	// REMOVE
	void removeNode( int idx );	
	void removeEdge(int fromNode, int toNode);

	// GET
    int nodeIndex(QString property_name, QString property_value);
	QSet<int> adjNodes(int index);
	QString nodeType(int index);

	int numNodes(){ return nodes.size(); }
	int numEdges(EdgeType t);
	int numSheets();
	int numCurves();

	QVector<int> getSheets();
	QVector<int> getCurves();

	// State
	GraphState State();
	void printState();
	void printNodeInfo(int index);
	GraphState difference(GraphState & other);
	operator GraphState() { return State(); }
	QVector<int> valences(bool isPrint = false);
	bool sameValences(DynamicGraph & other);
	Structure::Graph * toStructureGraph();

	// Graph edit
	QVector<DynamicGraph> candidateNodes(DynamicGraph & targetGraph);
	QVector<DynamicGraph> candidateEdges(DynamicGraph & targetGraph);

public:

	// Properties
	Structure::Graph * mGraph;
	int uniqueID;

	QMap<int, SimpleNode> nodes;
	QMap<int, SimpleEdge> edges;
	QMap<int, QSet<SimpleEdge> > adjacency;
};
