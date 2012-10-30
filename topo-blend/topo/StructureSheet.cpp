#include "StructureSheet.h"
using namespace Structure;

Sheet::Sheet(NURBSRectangle sheet, QString sheetID, QColor color)
{
    this->surface = sheet;
    this->id = sheetID;

    this->vis_property["color"] = color;
    this->vis_property["showControl"] = true;
}

QString Sheet::type()
{
    return SHEET;
}

QBox3D Sheet::bbox(double scaling)
{
    QBox3D box;

    foreach(std::vector<Vec3d> cps, surface.mCtrlPoint)
        foreach(Vec3d cp, cps)
            box.unite(cp);

	// Scaling
	QVector3D diagonal = box.size() * 0.5;
	box.unite(box.center() + (diagonal * scaling));
	box.unite(box.center() - (diagonal * scaling));

    return box;
}

std::vector<int> Sheet::controlCount()
{
	std::vector<int> count;

	count.push_back(surface.mNumUCtrlPoints);
	count.push_back(surface.mNumVCtrlPoints);

	return count;
}

std::vector<Vector3> Sheet::controlPoints()
{
	std::vector<Vector3> cpoints;

	for(int u = 0; u < surface.mNumUCtrlPoints; u++)
	{
		for(int v = 0; v < surface.mNumVCtrlPoints; v++)
		{
			cpoints.push_back( surface.mCtrlPoint[u][v] );
		}
	}

	return cpoints;
}

std::vector<Scalar> Sheet::controlWeights()
{
	std::vector<Scalar> cpoints;

	for(int v = 0; v < surface.mNumVCtrlPoints; v++)
		for(int u = 0; u < surface.mNumUCtrlPoints; u++)
			cpoints.push_back( surface.mCtrlWeight[u][v] );

	return cpoints;
}

void Structure::Sheet::get( const Vector3& coordinates, Vector3 & pos, std::vector<Vector3> & frame )
{
	double u = coordinates[0];
	double v = coordinates[1];

	frame.resize(3, Vector3(0));

	surface.Get(u, v, pos, frame[0], frame[1]);

	surface.GetFrame(u, v, pos, frame[0], frame[1], frame[2]);
}

Vec2d Structure::Sheet::approxCoordinates( const Vector3 & pos )
{
	return surface.timeAt( pos );
}

SurfaceMeshTypes::Vector3 Structure::Sheet::approxProjection( const Vector3 & point )
{
	Vector3 pos(0);
	Vec2d coords = approxCoordinates(point);
	surface.Get(coords[0], coords[1], pos);
	return pos;
}

std::vector< std::vector<Vector3> > Structure::Sheet::discretized(Scalar resolution)
{
	return surface.generateSurfaceTris( resolution );
}

Vector3 & Structure::Sheet::controlPoint( int idx )
{
    // TODO: return actual point
	int u = 0;
	int v = 0;
	return surface.mCtrlPoint[u][v];
}

void Sheet::draw()
{
    NURBS::SurfaceDraw::draw( &surface, vis_property["color"].value<QColor>(), vis_property["showControl"].toBool() );
}
