#pragma once

#include "StructureGraph.h"
using namespace Structure;

#include "Octree.h"

struct ParameterCoord{
	double u, v;
	double theta, psi;

	ParameterCoord(){ u = v = -1; theta = psi = 0; }
	ParameterCoord(double theta, double psi, double u, double v = 0){
		this->u = u;
		this->v = v;

		this->theta = theta;
		this->psi = psi;
	}

	bool operator < (const ParameterCoord& other) const{
		return (u < other.u);
	}
};

struct Synthesizer{
	static QVector<ParameterCoord> genFeatureCoordsCurve( Structure::Curve * curve );
	static QVector<ParameterCoord> genUniformCoordsCurve( Structure::Curve * curve, double resolution = -1);
	static QVector<ParameterCoord> genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points, double resolution = -1 );

	static QVector<ParameterCoord> genFeatureCoordsSheet( Structure::Sheet * sheet );
	static QVector<ParameterCoord> genUniformCoordsSheet( Structure::Sheet * sheet, double resolution = -1);
	static QVector<ParameterCoord> genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vec3d> & points, double resolution = -1 );

	static void computeOffsetsCurve( QVector<ParameterCoord> & samples, Structure::Curve * curve, QVector<double> &offsets, QVector<Vector3> &normals);
	static void computeOffsetsSheet( QVector<ParameterCoord> & samples, Structure::Sheet * sheet, QVector<double> &offsets, QVector<Vector3> &normals );

	static void synthesizeCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QString id = "" );

	static Vec3d intersectionPoint( Ray ray, Octree * useTree, int * faceIndex = 0 );
};
