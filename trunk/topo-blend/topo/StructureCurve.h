#pragma once

#include "StructureNode.h"
#include "NURBSCurve.h"

namespace Structure{

struct Curve : public Node
{
    // Constructors
    Curve(NURBSCurve newCurve, QString newID, QColor color = qRandomColor());

    // Underlaying structure
    NURBSCurve curve;

    // Properties
    QString type();
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();

	// Coordinates
	void get( const Vector3& coordinates, Vector3 & pos, std::vector<Vector3> & frame = noFrame() );
	Vector3 approxProjection( const Vector3 & pos );

    // Connections


    // Visualization
    void draw();
    void draw2D();
};

}
