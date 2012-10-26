#include "StructureCurve.h"
#include "LineSegment.h"
using namespace Structure;

Curve::Curve(NURBSCurve newCurve, QString newID, QColor color)
{
    this->curve = newCurve;
    this->id = newID;

    this->vis_property["color"] = color;
    this->vis_property["showControl"] = false;
}

QString Curve::type()
{
    return CURVE;
}

QBox3D Curve::bbox(double scaling)
{
    QBox3D box;

    foreach(Vec3d cp, curve.getControlPoints())
        box.unite(cp);

	// Scaling
	QVector3D diagonal = box.size() * 0.5;
	box.unite(box.center() + (diagonal * scaling));
	box.unite(box.center() - (diagonal * scaling));

    return box;
}

std::vector<int> Curve::controlCount()
{
	return std::vector<int>( 1, curve.GetNumCtrlPoints() );
}

std::vector<Vector3> Curve::controlPoints()
{
	return curve.getControlPoints();
}

std::vector<Scalar> Curve::controlWeights()
{
	return curve.getControlWeights();
}

void Structure::Curve::get( const Vector3& coordinates, Vector3 & pos, std::vector<Vector3> & frame )
{
	double u = coordinates[0];
	Vector3 der1(0);

	frame.resize(3, Vector3(0));

	curve.GetFrame(u, pos, frame[0], frame[1], frame[2]);
}

Vec2d Structure::Curve::approxCoordinates( const Vector3 & pos )
{
	Scalar t = curve.timeAt( pos );
	return Vec2d( t, 0 );
}

SurfaceMeshTypes::Vector3 Structure::Curve::approxProjection( const Vector3 & point )
{
	Vec2d coords = approxCoordinates(point);
	return curve.GetPosition(coords[0]);
}

std::vector< std::vector<Vector3> > Structure::Curve::discretized(Scalar resolution)
{
	return curve.toSegments( resolution );
}

Vector3 & Structure::Curve::controlPoint( int idx )
{
	return curve.mCtrlPoint[idx];
}

void Curve::draw()
{
    NURBS::CurveDraw::draw( &curve, vis_property["color"].value<QColor>(), vis_property["showControl"].toBool() );
}

