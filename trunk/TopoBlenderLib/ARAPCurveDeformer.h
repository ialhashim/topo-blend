// Adapted from code by Shaoting Zhang
// http://sourceforge.net/projects/meshtools/
#pragma once

#include "SurfaceMeshHelper.h"

extern int globalArapIterations;
extern int globalArapSize;

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
#include <Eigen/SVD>
#include <Eigen/Geometry>
using namespace Eigen;

class ARAPCurveDeformer{

public:
	ARAPCurveDeformer( std::vector<Vec3d> curvePoints, int sizeNeighborhood = 1 );

    std::vector<Vec3d> points;

private:
    void ComputeCotWeights();
    void BuildAndFactor();
    void SVDRotation();

public:
    void Deform(int ARAPIteration = 1);

private:
    std::vector<Matrix3d> R;
    std::vector<Vector3d> OrigMesh;
    std::vector<VectorXd> xyz;
    std::vector<VectorXd> b;

    // Frequently used
    int nVerts;
    std::vector< std::map<int, double> > wij_weight;
    std::vector< bool > isAnchorPoint, isControlPoint;
	int neighborhoodSize;

    SparseMatrix<double> At;
    CholmodSupernodalLLT< SparseMatrix<double> > solver;
    bool isSolverReady;

	std::vector<int> neighbors( int idx );

public:

    // Control points
    void SetAnchor( int v ){
        isAnchorPoint[v] = true;
    }

	void setControl(int v){
		isControlPoint[v] = true;
		isSolverReady = false;
	}

    void UpdateControl( int v, const Vec3d & newPos ){
        points[v] = newPos;
    }

    void ClearAnchors(){
        isAnchorPoint.clear();
        isAnchorPoint.resize(nVerts, false);
        isSolverReady = false;
    }

    void ClearControl(){
        isControlPoint.clear();
        isControlPoint.resize(nVerts, false);
        isSolverReady = false;
    }

    void ClearAll(){
        ClearAnchors();
        ClearControl();

		wij_weight.resize(nVerts);
    }

	void MakeReady(){
		this->Deform(2);
	}
};
