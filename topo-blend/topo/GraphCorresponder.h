#pragma once;

#include "SurfaceMeshTypes.h"
#include "StructureGraph.h"

#include <vector>

class GraphCorresponder
{
public:
    GraphCorresponder(Structure::Graph *source, Structure::Graph *target);

	// Hausdorff distance
	float supinfDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);
	float HausdorffDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);

	// Distance matrix
	void computeDistanceMatrix();
	void normalizeDistanceMatrix();
	bool minElementInMatrix(std::vector< std::vector<float> > &M, int &row, int &column);
	void visualizeHausdorffDistances(int sourceID);

	// Find correspondences
	void findCorrespondences();

private:
	Structure::Graph *sg, *tg;
	std::vector< std::vector<float> > disM;
};

