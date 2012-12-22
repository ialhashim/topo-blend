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

	void computeDistances( Vector3 startingPoint, double resolution = 0.25 );
	void computeDistances( std::vector<Vector3> startingPoints, double resolution = 0.25 );
	double distanceTo( Vector3 point, std::vector<Vector3> & path = std::vector<Vector3>() );
	Structure::Node * closestNeighbourNode( Vector3 to, double resolution = 0.25 );
	void clear();

	Structure::Graph * g;

	adjacency_list_t adjacency_list;
	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;

	std::map< Structure::Node *, std::vector<GraphDistanceNode> > nodesMap;
	std::map< Structure::Node *, std::vector<Vector3> > samplePoints;
	std::map< Structure::Node *, std::pair<int,int> > nodeCount;

	std::vector<Vector3> allPoints;
	std::vector<double> dists;
	std::vector<Structure::Node *> correspond;
	std::set< std::pair<int,int> > jumpPoints;

	bool isReady;

	// DEBUG:
	void draw();
};
