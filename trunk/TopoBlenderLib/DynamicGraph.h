#pragma once
#include "StructureGraph.h"

#include "DynamicGraphGlobal.h"

typedef QPair<int,int> QPairInt;

namespace DynamicGraphs
{
	class DynamicGraph
	{
	public:

		DynamicGraph(Structure::Graph * fromGraph = 0);
		DynamicGraph clone();

		// ADD
		int addNode(Properties properties = noProperties(), int index = -1);
		int addEdge(int fromNode, int toNode);

		int cloneNode(int index, bool cloneEdges = false);

		// REMOVE
		void removeNode( int idx );	
		void removeEdge(int fromNode, int toNode);

		// GET
		int nodeIndex(QString property_name, QVariant property_value);
		std::vector<int> nodesWith(QString property_name, QVariant property_value);
		QSet<int> adjNodes(int index);
		QString nodeType(int index);
		SimpleNode * getNode( QString originalID );
		Structure::Link * getOriginalLink( QString originalID1, QString originalID2 );
		QMap<int, SimpleEdge> getEdges( int nodeIDX );
		bool hasEdge(int n1_index, int n2_index);

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

		// Flags
		void flagNodes(QString propertyName, QVariant value);
		QVector<QVariant> flags(QString propertyName);

		// Valence based operations
		int valence(int nodeIndex);
		QVector< QPairInt > valences(bool isPrint = false);
		bool sameValences( DynamicGraph & other );
		QVector< QPairInt > correspondence( DynamicGraph & other, double & score, bool isPrint = false );
		//void correspondTo( DynamicGraph & other );

		// Generate structure graph
		Structure::Graph * toStructureGraph();

		// Graph edit
		QVector<DynamicGraph> candidateNodes(DynamicGraph & targetGraph);
		QVector<DynamicGraph> candidateEdges(DynamicGraph & targetGraph);

	public:

		// Properties
		Structure::Graph * mGraph;
		Structure::Graph * graph(){ return mGraph; }

		int uniqueID;
		int uniqueEdgeID;

		QMap<int, SimpleNode> nodes;
		QMap<int, SimpleEdge> edges;
		QMap<int, QSet<SimpleEdge> > adjacency;

		// Special cases
		QMap< int, QMap< int, std::vector<Vec4d> > > specialCoords;
		QMap< int, int > movable;
		QMap< int, std::pair<int, std::pair<int,double> > > growingCurves;
		QMap< int, std::pair<int,Array2D_Vector3> > growingSheets;
		std::vector<Vec4d> firstSpecialCoord( int node_index );
	};

}
