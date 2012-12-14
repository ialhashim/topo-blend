#pragma once
#include "StructureGraph.h"
#include "Dijkstra.h"

struct GraphDistanceNode{
	Vector3 pos;
	Structure::Node * n;
	int idx, gid;
	GraphDistanceNode(Vector3 & position, Structure::Node * node, int index, int globalID):
		pos(position),n(node),idx(index),gid(globalID){}
};

class GraphDistance
{
public:
    GraphDistance(Structure::Graph * graph);

	void computeDistances( Structure::Node * startNode, const Vec4d & coords = Vec4d(0), double resolution = 0.1 );
    Structure::Graph * g;

	adjacency_list_t adjacency_list;

	std::map< Structure::Node *, std::vector<GraphDistanceNode> > nodesMap;
	std::map< Structure::Node *, std::vector<Vector3> > samplePoints;
	std::map< Structure::Node *, std::pair<int,int> > nodeCount;

	std::vector<Vector3> allPoints;
	std::vector<double> dists;

	bool isReady;

	// DEBUG:
	void draw();
	void dbgPoint(const Vec3d & p);
	void pushDebugPoints(const std::vector<Vec3d> & pnts);
	std::vector< std::vector<Vec3d> > debugPoints;
	std::vector< QColor > debugColors;
};
