#pragma once
#include "StructureGraph.h"

class GraphDistance
{
public:
    GraphDistance(Structure::Graph * graph);

	void computeDistances(Structure::Node * n, const Vec4d & coords = Vec4d(0));

    Structure::Graph * g;

	std::vector<Vector3> cpnts;
	std::vector<double> dists;

	// DEBUG:
	void draw();
	void dbgPoint(const Vec3d & p);
	void pushDebugPoints(const std::vector<Vec3d> & pnts);
	std::vector< std::vector<Vec3d> > debugPoints;
	std::vector< QColor > debugColors;

	std::vector< Vec3d> test_pnt;
	std::vector< double > test_dist;
};

