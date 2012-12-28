#pragma once;

#include "SurfaceMeshTypes.h"
#include "StructureGraph.h"

#include <vector>

typedef std::pair<std::set<QString>, std::set<QString>> SET2SET;

class GraphCorresponder
{
public:
    GraphCorresponder(Structure::Graph *source, Structure::Graph *target);

	// Hausdorff distance
	float supInfDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);
	float HausdorffDistance(std::vector<Vector3> &A, std::vector<Vector3> &B);

	// Distance matrices
	void computeHausdorffDistanceMatrix(std::vector< std::vector<float> > & M);
	void computeDegreeDifferenceMatrix(std::vector< std::vector<float> > & M);
	void computeDistanceMatrix();
	void initializeMatrix(std::vector< std::vector<float> > & M);
	void normalizeMatrix(std::vector< std::vector<float> > & M);

	// Visualization
	void visualizePart2PartDistance(int sourceID);

	// Find correspondences
	bool minElementInMatrix(std::vector< std::vector<float> > &M, int &row, int &column);
	void findOneToOneCorrespondences();
	void findOneToManyCorrespondences();
	void correspondTwoNodes(Structure::Node *sNode, Structure::Node *tNode);
	void correspondTwoCurves(Structure::Curve *sCurve, Structure::Curve *tCurve);
	void correspondTwoSheets(Structure::Sheet *sSheet, Structure::Sheet *tSheet);

	// The main access
	void computeCorrespondences();

	// Result
	std::vector<SET2SET> correspondences;

private:
	Structure::Graph *sg, *tg;
	std::vector< std::vector<float> > disM;
};