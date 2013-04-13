#pragma once

#include "StructureNode.h"
#include "NURBSRectangle.h"

namespace Structure{

struct Sheet : public Node
{
    // Constructors
    Sheet(const NURBS::NURBSRectangled & sheet, QString sheetID, QColor color = qRandomColor());
	Node * clone();

    // Underlaying structure
    NURBS::NURBSRectangled surface;

    // Properties
    QString type();
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();
	Vector3 & controlPoint(int idx);
	void setControlPoints(const std::vector<Vector3> & newPositions);
	int numCtrlPnts();

	// Modifiers
	void moveBy( const Vec3d & delta );
	void scale( Scalar scaleFactor );
	void rotate( double angle, Vector3 axis );
    std::vector< std::vector<Vec3d> > foldTo( const std::vector<Vec4d> & curve, bool isApply = false );
	void equalizeControlPoints( Structure::Node * other );
	void deformTo( const Vec4d & handle, const Vector3 & to, bool isRigid );

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);
	std::vector< std::vector<Vec4d> > discretizedPoints(Scalar resolution);

	// Coordinates
    void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame );
	Vector3 position( const Vec4d& coordinates );
	Vec4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Geometric properties
	Scalar area();

    // Connections

    // Visualization
    void draw(bool isShowCtrlPts = false);
	void drawWithNames(int nID, int pointIDRange);
};

}
