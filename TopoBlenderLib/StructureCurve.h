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
    QBox3D bbox(double scaling = 1.0);

	std::vector<int> controlCount();
	std::vector<Vector3> controlPoints();
	std::vector<Scalar> controlWeights();

	Vector3 & controlPoint( int idx );
	Vector3 & controlPointFromCoord( Vec4d coord );
	int controlPointIndexFromCoord( Vec4d coord );
	void setControlPoints(const std::vector<Vector3> & newPositions);
	void laplacianSmoothControls( int num_iterations, std::set<int> anchored = std::set<int>() );

	int numCtrlPnts();

	// Modifiers
	void moveBy( const Vec3d & delta );
	void scale( Scalar scaleFactor );
	void rotate( double angle, Vector3 axis );
	std::vector<Vec3d> foldTo( Vec4d & foldPoint, bool isApply = false );
	void refineControlPoints(int nU, int nV = 0);
	void equalizeControlPoints( Structure::Node * other );
	void deformTo( const Vec4d & handle, const Vector3 & to, bool isRigid );
	void deformTwoHandles( Vec4d handleA, Vector3 newPosA, Vec4d handleB, Vector3 newPosB );

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);
	std::vector< std::vector<Vec4d> > discretizedPoints(Scalar resolution);

	// Coordinates
    void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame );
	Vector3 position( const Vec4d& coordinates );
	Vec4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Encoding
	static CurveEncoding encodeCurve( Array1D_Vector3 points, Vector3 start, Vector3 end, bool isFlip = false );
	static CurveEncoding encodeCurve( Curve * curve, Vector3 start, Vector3 end, bool isFlip = false );
	static Array1D_Vector3 decodeCurve( CurveEncoding cpCoords, Vector3 start, Vector3 end, double T = 1.0 );

	// Geometric properties
	Scalar area();
	Vec3d direction();

    // Connections

    // Visualization
    void draw(bool isShowCtrlPts = false);
	void drawWithNames(int nID, int pointIDRange);
};

}
