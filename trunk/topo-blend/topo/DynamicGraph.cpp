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

QVector<int> DynamicGraph::getSheets()
{
	QVector<int> sheets;
	foreach(SimpleNode n, nodes) 
		if(nodeType(n.idx) == Structure::SHEET)
			sheets.push_back(n.idx);
	return sheets;
}

int DynamicGraph::numSheets()
{
	return getSheets().size();
}

QVector<int> DynamicGraph::getCurves()
{
	QVector<int> curves;
	foreach(SimpleNode n, nodes) 
		if(nodeType(n.idx) == Structure::CURVE)
			curves.push_back(n.idx);
	return curves;
}

int DynamicGraph::numCurves()
{
	return getCurves().size();
}

int DynamicGraph::cloneNode( int index, bool cloneEdges )
{
	int newIndex = addNode( nodes[index].property, uniqueID );

	if(cloneEdges){
		QSet<int> adj = adjNodes( index );

		foreach(int ni, adj)
			addEdge(newIndex, ni);
	}

	return newIndex;
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

QVector<DynamicGraph> DynamicGraph::candidates(GraphState futureState)
{
	QVector<DynamicGraph> candidate;

	// Compute graph difference
	GraphState diff = difference( futureState );

	// Should return no candidates when they are exactly the same
	if( diff.isZero() ) return candidate;

	// Missing nodes: nodes have precedent
	if(diff.numSheets != 0 || diff.numCurves != 0)
	{
		QVector<int> activeNodes;
		bool isAdding = true;

		// Sheets and curves: sheet have precedent
		if(diff.numSheets != 0)
		{
			activeNodes = getSheets();
			if(diff.numSheets > 0) isAdding = false;
		}
		else if(diff.numCurves != 0)
		{
			activeNodes = getCurves();
			if(diff.numCurves > 0) isAdding = false;
		}

		// Generate candidates
		foreach(int ci, activeNodes)
		{
			DynamicGraph g = clone();

			if(isAdding)	
				g.cloneNode(ci);
			else
				g.removeNode(ci);

			candidate.push_back(g);
		}
	}

	// Edges
	else if(diff.numEdges() != 0)		// NOTICE: else-if, in order to generate gradual candidates
	{
		QVector<int> groupA, groupB;
		QSet<SimpleEdge> exploredEdges;
		bool isAdding = true;

		// Add existing edges as explored
		foreach(SimpleEdge e, edges)
			exploredEdges.insert(e);

		if(diff.numMixedEdges != 0)
		{
			// Curve-Sheet edges
			if(diff.numMixedEdges > 0) isAdding = false;
			groupA = getSheets();
			groupB = getCurves();
		}
		else if(diff.numSheetEdges != 0)
		{
			// Sheet-Sheet edges
			if(diff.numSheetEdges > 0) isAdding = false;
			groupA = getSheets();
			groupB = groupA;
		}
		else if(diff.numCurveEdges != 0)
		{
			// Curve-Curve edges
			if(diff.numCurveEdges > 0) isAdding = false;
			groupA = getCurves();
			groupB = groupA;
		}

		// Permutations
		foreach(int a, groupA)
		{
			foreach(int b, groupB)
			{
				SimpleEdge e(a, b);

				// Uniqueness check:
				if(a == b || exploredEdges.contains(e)) continue;
				exploredEdges.insert(e);

				// Add operation as a candidate graph
				DynamicGraph g = clone();

				if(isAdding)
					g.addEdge(a, b);
				else
					g.removeEdge(a, b);

				candidate.push_back(g);
			}
		}
	}

	return candidate;
}

Structure::Graph * DynamicGraph::toStructureGraph()
{
	Structure::Graph * graph = new Structure::Graph();
	QMap<int,QString> nodeMap;

	int id = 0;

	foreach(SimpleNode n, nodes)
	{
		if(nodeType(n.idx) == Structure::SHEET)
		{
			Structure::Sheet * s = (Structure::Sheet *) mGraph->getNode(n.property["original"]);
			QString nodeId = QString("%1-%2").arg(s->id).arg(QString::number(id++));
			graph->addNode( new Structure::Sheet(s->surface, nodeId) );
			nodeMap[n.idx] = nodeId;
		}

		if(nodeType(n.idx) == Structure::CURVE)
		{
			Structure::Curve * s = (Structure::Curve *) mGraph->getNode(n.property["original"]);
			QString nodeId = QString("%1-%2").arg(s->id).arg(QString::number(id++));
			graph->addNode( new Structure::Curve(s->curve, nodeId) );
			nodeMap[n.idx] = nodeId;
		}
	}

	foreach(SimpleEdge e, edges)
	{
		graph->addEdge( nodeMap[ e.n[0] ], nodeMap[ e.n[1] ] );
	}

	return graph;
}
