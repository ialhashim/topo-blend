#pragma once

#include "StructureNode.h"
#include "NURBSRectangle.h"

namespace Structure{

struct Sheet : public Node
{
    // Constructors
    Sheet(NURBSRectangle sheet, QString sheetID, QColor color = qRandomColor());

    // Underlaying structure
    NURBSRectangle surface;

    // Properties
    QString type();
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);

	// Coordinates
	void get( const Vector3& coordinates, Vector3 & pos, std::vector<Vector3> & frame = noFrame() );
	Vector3 approxProjection( const Vector3 & pos );

    // Connections

    // Visualization
    void draw();
};

}
