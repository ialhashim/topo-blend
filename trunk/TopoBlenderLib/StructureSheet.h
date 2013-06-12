#pragma once

#include "StructureNode.h"
#include "NURBSCurve.h"
#include "NURBSRectangle.h"

typedef QMap<int, Array1D_Real> SheetEncoding;

namespace Structure{

struct Sheet : public Node
{
    // Constructors
    Sheet(const NURBS::NURBSRectangled & sheet, QString sheetID, QColor color = qRandomColor2());
	Node * clone();

    // Underlaying structure
    NURBS::NURBSRectangled surface;

    // Properties
    QString type();
    Eigen::AlignedBox3d bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();
	Vector3 & controlPoint(int idx);
	void setControlPoints(const std::vector<Vector3> & newPositions);
	int numCtrlPnts();
	int numUCtrlPnts();
	int numVCtrlPnts();

	// Modifiers
	void moveBy( const Vector3d & delta );
	void scale( Scalar scaleFactor );
	void rotate( double angle, Vector3 axis );
    std::vector< std::vector<Vector3d> > foldTo( const Array1D_Vector4d & curve, bool isApply = false );
	void refineControlPoints(int nU, int nV = 0);
	void equalizeControlPoints( Structure::Node * other );
	void deformTo( const Vector4d & handle, const Vector3 & to, bool isRigid );
	void deformTwoHandles( Vector4d& handleA, Vector3 newPosA, Vector4d& handleB, Vector3 newPosB );
	NURBS::NURBSCurved convertToNURBSCurve(Vector3d p, Vector3d dir); 

	Array2D_Vector3 discretized(Scalar resolution);
	Array2D_Vector4d discretizedPoints(Scalar resolution);

	// Coordinates
    void get( const Vector4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame );
	Vector3 position( const Vector4d& coordinates );
	Vector4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Geometric properties
	Scalar area();

	// Encode and decode
	static SheetEncoding encodeSheet( Sheet * sheet, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z );
	static Array1D_Vector3 decodeSheet( SheetEncoding cpCoords, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z );

    // Connections

    // Visualization
    void draw(bool isShowCtrlPts = false);
	void drawWithNames(int nID, int pointIDRange);
};

}
