#pragma once

#include "StructureNode.h"
#include "NURBSRectangle.h"

namespace Structure{

struct Sheet : public Node
{
    // Constructors
    Sheet(const NURBSRectangle & sheet, QString sheetID, QColor color = qRandomColor());
	Node * clone();

    // Underlaying structure
    NURBSRectangle surface;

    // Properties
    QString type();
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();
	Vector3 & controlPoint(int idx);

	// Modifiers
	void moveBy( const Vec3d & delta );
	std::vector< std::vector<Vec3d> > Sheet::foldTo( const std::vector<Vec4d> & curve, bool isApply = false );

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);
	std::vector< std::vector<Vec4d> > discretizedPoints(Scalar resolution);

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

	std::vector<Vector3> debugPoints;
};

}
