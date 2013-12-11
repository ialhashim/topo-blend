#pragma once
#include "RelationDetector.h"
class GlobalReflectionSymmScorer :	public RelationDetector
{
public:
    GlobalReflectionSymmScorer(Structure::Graph* g, int ith, double normalizeCoef, bool bUsePart=false, int logLevel=0):RelationDetector(g, "GlobalReflectionSymmScorer-", ith, normalizeCoef, 1, logLevel)
    {
		bUsePart_ = bUsePart;
		if ( this->logLevel_ > 0)
			this->logStream_ << "normalize coeff: " << this->normalizeCoef_ << "\n";

		init();
    }

    double evaluate();
	double evaluate(const Eigen::Vector3d &center, const Eigen::Vector3d &normal, double maxGlobalSymmScore=1.0);
    
	Eigen::MatrixXd cpts_; // control point of the graph
    Eigen::Vector3d center_;// center of the graph by control points, i.e. of the reflection plane
	Eigen::Vector3d normal_; // normal of the reflection plane
	

    std::vector<Eigen::MatrixXd> nodesCpts_; 
    std::vector<Eigen::Vector3d> nodesCenter_; // center of each node by control points, if the diameter of the part is 0, it is not pushed back in this array.

private:
	void init();
	// compute min mean distance between pts and other parts in the graph.
	double minMeanDistance(Eigen::MatrixXd& pts, int& nodeNo);
	//todo find all possible reflection plane. Current implementation: use plane through center & paralleling with xy or xz plane, since z is up in all models.
	Eigen::Vector3d findReflectPlane(const Eigen::Vector3d& center);
	int findNearestPart(const Eigen::Vector3d &nc, const QString & type, double size, double& dist);

	bool bUsePart_;
};

