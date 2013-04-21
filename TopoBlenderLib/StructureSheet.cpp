// Used for sheet folding
#include "NanoKdTree.h"
#include "GraphDistance.h"
#include "PCA.h"

#if defined(Q_OS_MAC)
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include "StructureSheet.h"
using namespace Structure;

Sheet::Sheet(const NURBS::NURBSRectangled & sheet, QString sheetID, QColor color)
{
    this->surface = sheet;
	this->surface.quads.clear();

    this->id = sheetID;
    this->vis_property["color"] = color;
    this->vis_property["showControl"] = false;
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

	surface.GetFrame(u, v, pos, frame[0], frame[1], frame[2]);
}

SurfaceMesh::Vector3 Sheet::position( const Vec4d& coordinates )
{
    std::vector<Vector3> nf = noFrame();
    Vector3 p(0); get(coordinates,p, nf);
	return p;
}

Vec4d Sheet::approxCoordinates( const Vector3 & pos )
{
	return surface.timeAt( pos );
}

SurfaceMesh::Vector3 Sheet::approxProjection( const Vector3 & point )
{
	Vector3 pos(0);
	Vec4d coords = approxCoordinates(point);
    surface.Get(coords[0], coords[1], &pos);
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

SurfaceMesh::Scalar Sheet::area()
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

SurfaceMesh::Vector3 Sheet::center()
{
	Vector3 pos(0);
    std::vector<Vector3> nf = noFrame();
    get(Vec4d(0.5,0.5,0,0), pos, nf);
	return pos;
}

void Sheet::draw(bool isShowCtrlPts)
{
    NURBS::SurfaceDraw::draw( &surface, vis_property["color"].value<QColor>(), isShowCtrlPts );

	// Draw selections
	GLUquadricObj *quadObj = gluNewQuadric();

	gluQuadricDrawStyle(quadObj, GLU_FILL);
	gluQuadricNormals(quadObj, GLU_SMOOTH);

	foreach (int pID, selections.keys())
	{
		QColor color = selections[pID];
		glColor3d(color.red(), color.green(), color.blue());

		int u = pID / surface.mNumVCtrlPoints;
		int v = pID % surface.mNumVCtrlPoints;

		Vector3 p = surface.GetControlPoint(u, v);

		glPushMatrix();
		glTranslatef(p.x(), p.y(), p.z());
		gluSphere(quadObj, 0.02, 16, 16);
		glPopMatrix();
	}

	gluDeleteQuadric(quadObj);
}

void Sheet::drawWithNames( int nID, int pointIDRange )
{
	int pID = nID * pointIDRange;

    //float radius = 1.0f;
	glPointSize(20.0f);

	for(int u = 0; u < surface.mNumUCtrlPoints; u++)
	{
		for(int v = 0; v < surface.mNumVCtrlPoints; v++)
		{
			glPushName(pID++);

			Vec3d p = surface.GetControlPoint(u, v);

			glBegin(GL_POINTS);
			glVertex3d(p.x(), p.y(), p.z());
			glEnd();

			glPopName();
		}
	}
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

void Sheet::scale( Scalar scaleFactor )
{
	this->surface.scale(scaleFactor);

	this->surface.quads.clear();
}

void Sheet::rotate( double angle, Vector3 axis )
{
	angle *= 3.14159265358979 /180; 

	Array2D_Vector3 &mCtrlPoint = this->surface.mCtrlPoint;

	for(int y = 0; y < (int)mCtrlPoint.size(); y++)
		for(int x = 0; x < (int)mCtrlPoint[0].size(); x++)
			mCtrlPoint[y][x]  = rotatedVec(mCtrlPoint[y][x], angle, axis);

	this->surface.quads.clear();
}

void Sheet::equalizeControlPoints( Structure::Node * _other )
{
	Structure::Sheet *other = (Structure::Sheet*) _other;

	int nU = qMax(other->surface.mNumUCtrlPoints,surface.mNumUCtrlPoints);
	int nV = qMax(other->surface.mNumVCtrlPoints,surface.mNumVCtrlPoints);

	Array2D_Vector3 my_cp, other_cp;

	NURBS::NURBSRectangled mySurface = surface;
	NURBS::NURBSRectangled otherSurface = other->surface;

	int k = 0;

	// Refine U
	k = nU - mySurface.mNumUCtrlPoints;
	my_cp = mySurface.simpleRefine(k, 0);

	k = nU - otherSurface.mNumUCtrlPoints;
	other_cp = otherSurface.simpleRefine(k, 0);

	// Both now have same number of U control points
	mySurface = NURBS::NURBSRectangled::createSheetFromPoints(my_cp);
	otherSurface = NURBS::NURBSRectangled::createSheetFromPoints(other_cp);

	// Refine V
	k = nV - mySurface.mNumVCtrlPoints;
	my_cp = mySurface.simpleRefine(k, 1);

	k = nV - otherSurface.mNumVCtrlPoints;
	other_cp = otherSurface.simpleRefine(k, 1);

	// Both have exactly same number of control points
	surface = NURBS::NURBSRectangled::createSheetFromPoints(my_cp);
	other->surface = NURBS::NURBSRectangled::createSheetFromPoints(other_cp);
}

int Sheet::numCtrlPnts()
{
	return surface.mNumUCtrlPoints * surface.mNumVCtrlPoints;
}

void Sheet::deformTo( const Vec4d & handle, const Vector3 & to, bool isRigid )
{
	Vector3 p = position( handle );

	double diff = (p - to).norm();
	if(diff < 1e-7) return;

	//if( isRigid )
	{
		Vec3d delta = to - p;
		this->moveBy( delta );
		return;
	}
}

void Structure::Sheet::refineControlPoints( int nU, int nV /*= 0*/ )
{
	Array2D_Vector3 my_cp;

	NURBS::NURBSRectangled mySurface = surface;

	int k = 0;

	// Refine U
	k = nU - mySurface.mNumUCtrlPoints;
	my_cp = mySurface.simpleRefine(k, 0);
	mySurface = NURBS::NURBSRectangled::createSheetFromPoints(my_cp);

	// Refine V
	k = nV - mySurface.mNumVCtrlPoints;
	my_cp = mySurface.simpleRefine(k, 1);
	surface = NURBS::NURBSRectangled::createSheetFromPoints(my_cp);
}

int Structure::Sheet::numUCtrlPnts()
{
	return surface.mNumUCtrlPoints;
}

int Structure::Sheet::numVCtrlPnts()
{
	return surface.mNumVCtrlPoints;

}


NURBS::NURBSCurved Structure::Sheet::nurbCurve( Vec3d p, Vec3d dir )
{
	Vec3d uDir = surface.mCtrlPoint.front().back() - surface.mCtrlPoint.front().front();
	Vec3d vDir = surface.mCtrlPoint.back().front() - surface.mCtrlPoint.front().front();
	uDir.normalize(); vDir.normalize();

	double uDot = dot(uDir, dir);
	double vDot = dot(vDir, dir);

	std::vector<Vec3d> curvePoints;
	std::vector<double> curveWeights;

	// along U
	if (abs(uDot) > abs(vDot))
	{
		// get the row that closet to p
		double minDis = DBL_MAX;
		double minID = -1;
		std::vector<Vec3d> uPoints0 = surface.GetControlPointsU(0);
		for (int i = 0; i < (int)uPoints0.size(); i++)
		{
			double dis = (p - uPoints0[i]).norm();
			if (dis < minDis)	{minDis = dis;	minID = i;}
		}

		curvePoints = surface.GetControlPointsV(minID);
		curveWeights = surface.GetControlWeightsV(minID);

		if (uDot < 0)
		{
			std::reverse(curvePoints.begin(), curvePoints.end());
			std::reverse(curveWeights.begin(), curveWeights.end());
		}
	}
	// along V
	else
	{
		// get the row that closet to p
		double minDis = DBL_MAX;
		double minID = -1;
		std::vector<Vec3d> vPoints0 = surface.GetControlPointsV(0);
		for (int i = 0; i < (int)vPoints0.size(); i++)
		{
			double dis = (p - vPoints0[i]).norm();
			if (dis < minDis)	{minDis = dis;	minID = i;}
		}

		curvePoints = surface.GetControlPointsU(minID);
		curveWeights = surface.GetControlWeightsU(minID);

		if (vDot < 0)
		{
			std::reverse(curvePoints.begin(), curvePoints.end());
			std::reverse(curveWeights.begin(), curveWeights.end());
		}
	}

	return NURBS::NURBSCurved(curvePoints, curveWeights);
}
