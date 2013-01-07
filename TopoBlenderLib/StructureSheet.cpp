// Used for sheet folding
#include "NanoKdTree.h"
#include "GraphDistance.h"
#include "PCA.h"

#include "StructureSheet.h"
using namespace Structure;

Sheet::Sheet(const NURBSRectangle & sheet, QString sheetID, QColor color)
{
    this->surface = sheet;
	this->surface.quads.clear();

    this->id = sheetID;
    this->vis_property["color"] = color;
    this->vis_property["showControl"] = true;
}

Node * Sheet::clone()
{
	Sheet * cloneSheet = new Sheet( this->surface, this->id );
	cloneSheet->surface.quads.clear();
	cloneSheet->property = this->property;
	cloneSheet->vis_property = this->vis_property;
	return cloneSheet;
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

void Sheet::setControlPoints( const std::vector<Vector3> & newPositions )
{
	int i = 0;

	for(int u = 0; u < surface.mNumUCtrlPoints; u++)
		for(int v = 0; v < surface.mNumVCtrlPoints; v++)
			surface.mCtrlPoint[u][v] = newPositions[i++];
}

std::vector<Scalar> Sheet::controlWeights()
{
	std::vector<Scalar> cpoints;

	for(int v = 0; v < surface.mNumVCtrlPoints; v++)
		for(int u = 0; u < surface.mNumUCtrlPoints; u++)
			cpoints.push_back( surface.mCtrlWeight[u][v] );

	return cpoints;
}

void Sheet::get( const Vec4d& coordinates, Vector3 & pos, std::vector<Vector3> & frame )
{
	double u = coordinates[0];
	double v = coordinates[1];

	frame.resize(3, Vector3(0));

	surface.Get(u, v, pos, frame[0], frame[1]);
	surface.GetFrame(u, v, pos, frame[0], frame[1], frame[2]);
}

SurfaceMeshTypes::Vector3 Sheet::position( const Vec4d& coordinates )
{
	Vector3 p(0); get(coordinates,p);
	return p;
}

Vec4d Sheet::approxCoordinates( const Vector3 & pos )
{
	return surface.timeAt( pos );
}

SurfaceMeshTypes::Vector3 Sheet::approxProjection( const Vector3 & point )
{
	Vector3 pos(0);
	Vec4d coords = approxCoordinates(point);
	surface.Get(coords[0], coords[1], pos);
	return pos;
}

std::vector< std::vector<Vector3> > Sheet::discretized(Scalar resolution)
{
	return surface.generateSurfaceTris( resolution );
}

std::vector< std::vector<Vec4d> > Sheet::discretizedPoints( Scalar resolution )
{
	std::vector< std::vector<Vec4d> > coords;
	surface.generateSurfacePointsCoords(resolution, coords);
	return coords;
}

void Sheet::moveBy( const Vec3d & delta )
{
	surface.translate( delta );
}

Vector3 & Sheet::controlPoint( int idx )
{
	int u = idx / surface.mNumVCtrlPoints;
	int v = idx - (u * surface.mNumVCtrlPoints);
	return surface.mCtrlPoint[u][v];
}

SurfaceMeshTypes::Scalar Sheet::area()
{
	double factor = 0.5; // distance of first control points

	Scalar r = factor * (surface.mCtrlPoint[0][0] - surface.mCtrlPoint[0][1]).norm();
	std::vector< std::vector<Vector3> > tris = discretized(r);

	double a = 0;

	foreach(std::vector<Vector3> v, tris)
	{
		double triArea = 0.5 * cross((v[1] - v[0]), (v[2] - v[0])).norm();
		a += triArea;
	}

	return a;
}

SurfaceMeshTypes::Vector3 Sheet::center()
{
	Vector3 pos(0);
	get(Vec4d(0.5,0.5,0,0), pos);
	return pos;
}

void Sheet::draw()
{
    NURBS::SurfaceDraw::draw( &surface, vis_property["color"].value<QColor>(), vis_property["showControl"].toBool() );
}

std::vector< std::vector<Vec3d> > Sheet::foldTo( const std::vector<Vec4d> & curve, bool isApply )
{
	std::vector<Vec3d> projections, coords;
	NanoKdTree tree;
	foreach(Vec4d coord, curve) 
	{
		coords.push_back(Vec3d(coord[0], coord[1], 0));
		projections.push_back( surface.projectOnControl( coord[0], coord[1] ) );
		tree.addPoint( projections.back() );
	}
	tree.build();

	std::vector< std::vector<Vec3d> > deltas = std::vector< std::vector<Vec3d> > (surface.mNumUCtrlPoints, 
		std::vector<Vec3d>(surface.mNumVCtrlPoints, Vector3(0)));

	// Project onto itself (PCA then project on closest main axis)
	Vec3d mp = PCA::mainAxis(coords);
	//int midIdx = curve.size() * 0.5;

	for(int u = 0; u < surface.mNumUCtrlPoints; u++)
	{
		for(int v = 0; v < surface.mNumVCtrlPoints; v++)
		{
			Vec4d currCoord(double(u) / (surface.mNumUCtrlPoints-1), double(v) / (surface.mNumVCtrlPoints-1),0,0);

			double minDist = DBL_MAX;
			int minIDX = -1;

			for(int i = 0; i < (int)curve.size(); i++)
			{
				Vec4d coord = curve[i];

				if(qMax(mp[0],mp[1]) == mp[0])
					currCoord[1] = coord[1];
				else
					currCoord[0] = coord[0];

				double dist = (coord - currCoord).norm();

				if(dist < minDist)
				{
					minIDX = i;
					minDist = dist;
				}
			}

			deltas[u][v] = surface.mCtrlPoint[u][v] - projections[minIDX];
		}
	}

	if(isApply)
	{
		for(int u = 0; u < surface.mNumUCtrlPoints; u++)
		{
			for(int v = 0; v < surface.mNumVCtrlPoints; v++)
			{
				surface.mCtrlPoint[u][v] += -deltas[u][v];
			}
		}
	}

	return deltas;
}

void Structure::Sheet::scale( Scalar scaleFactor )
{
	this->surface.scale(scaleFactor);
}
