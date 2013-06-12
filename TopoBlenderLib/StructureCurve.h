#pragma once

#include "StructureNode.h"
#include "NURBSCurve.h"

typedef QMap<int, Array1D_Real> CurveEncoding;

namespace Structure{

struct Curve : public Node
{
    // Constructors
    Curve(const NURBS::NURBSCurved & newCurve, QString newID, QColor color = qRandomColor2());
	Node * clone();

    // Underlaying structure
    NURBS::NURBSCurved curve;

    // Properties
    QString type();
    Eigen::AlignedBox3d bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();

	Vector3 & controlPoint( int idx );
	Vector3 & controlPointFromCoord( Vector4d& coord );
	int controlPointIndexFromCoord( Vector4d& coord );
	void setControlPoints(const std::vector<Vector3> & newPositions);
	void laplacianSmoothControls( int num_iterations, std::set<int> anchored = std::set<int>() );

	int numCtrlPnts();

	// Modifiers
	void moveBy( const Vector3d & delta );
	void scale( Scalar scaleFactor );
	void rotate( double angle, Vector3 axis );
	std::vector<Vector3d> foldTo( Vector4d & foldPoint, bool isApply = false );
	void refineControlPoints(int nU, int nV = 0);
	void equalizeControlPoints( Structure::Node * other );
	void deformTo( const Vector4d & handle, const Vector3 & to, bool isRigid );
	void deformTwoHandles( Vector4d& handleA, Vector3 newPosA, Vector4d& handleB, Vector3 newPosB );

	Array2D_Vector3 discretized(Scalar resolution);
	Array2D_Vector4d discretizedPoints(Scalar resolution);

	// Coordinates
    void get( const Vector4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame );
	Vector3 position( const Vector4d& coordinates );
	Vector4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Encoding
	static CurveEncoding encodeCurve( Array1D_Vector3 points, Vector3 start, Vector3 end, bool isFlip = false );
	static CurveEncoding encodeCurve( Curve * curve, Vector3 start, Vector3 end, bool isFlip = false );
	static Array1D_Vector3 decodeCurve( CurveEncoding cpCoords, Vector3 start, Vector3 end, double T = 1.0 );

	// Geometric properties
	Scalar area();
	Vector3d direction();

    // Connections

    // Visualization
    void draw(bool isShowCtrlPts = false);
	void drawWithNames(int nID, int pointIDRange);
};

}
