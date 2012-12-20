#pragma once

#include "StructureNode.h"
#include "NURBSCurve.h"

namespace Structure{

struct Curve : public Node
{
    // Constructors
    Curve(const NURBSCurve & newCurve, QString newID, QColor color = qRandomColor());
	Node * clone();

    // Underlaying structure
    NURBSCurve curve;

    // Properties
    QString type();
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();

	Vector3 & controlPoint( int idx );
	Vector3 & controlPointFromCoord( Vec4d coord );
	int controlPointIndexFromCoord( Vec4d coord );
	void laplacianSmoothControls( int num_iterations, std::set<int> anchored = std::set<int>() );

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);
	std::vector< std::vector<Vector3> > discretizedPoints(Scalar resolution);

	// Coordinates
	void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame = noFrame() );
	Vector3 position( const Vec4d& coordinates );
	Vec4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Geometric properties
	Scalar area();

    // Connections

    // Visualization
    void draw();
};

}
