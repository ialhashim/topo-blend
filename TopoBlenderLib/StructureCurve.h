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
	void setControlPoints(const std::vector<Vector3> & newPositions);
	void laplacianSmoothControls( int num_iterations, std::set<int> anchored = std::set<int>() );

	int numCtrlPnts();

	// Modifiers
	void moveBy( const Vec3d & delta );
	void scale( Scalar scaleFactor );
	void rotate( double angle, Vector3 axis );
	std::vector<Vec3d> foldTo( Vec4d & foldPoint, bool isApply = false );
	void equalizeControlPoints( Structure::Node * other );

	std::vector< std::vector<Vector3> > discretized(Scalar resolution);
	std::vector< std::vector<Vec4d> > discretizedPoints(Scalar resolution);

	// Coordinates
    void get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame );
	Vector3 position( const Vec4d& coordinates );
	Vec4d approxCoordinates( const Vector3 & pos );
	Vector3 approxProjection( const Vector3 & point );
	Vector3 center();

	// Encoding
	void encodeShape();
	void decodeShape(QVector<Vec3d> & deltas);


	// Geometric properties
	Scalar area();

    // Connections

    // Visualization
    void draw();
	void drawWithNames(int nID, int pointIDRange);
};

}
