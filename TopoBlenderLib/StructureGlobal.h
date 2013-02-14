#pragma once

#include <assert.h>
#include <QVariant>
#include <QMap>
#include <QSet>

#include "SurfaceMeshHelper.h"
#include "SurfaceMeshTypes.h"
#include "NurbsDraw.h"
#include "../CustomDrawObjects.h"

#undef max

static inline std::vector<Vector3> noFrame(){
	return std::vector<Vector3>();
}

static inline Vector3 orthogonalVector(const Vector3& n) {
	if ((abs(n.y()) >= 0.9 * abs(n.x())) &&
		abs(n.z()) >= 0.9 * abs(n.x())) return Vector3(0.0, -n.z(), n.y());
	else if ( abs(n.x()) >= 0.9 * abs(n.y()) &&
		abs(n.z()) >= 0.9 * abs(n.y()) ) return Vector3(-n.z(), 0.0, n.x());
	else return Vector3(-n.y(), n.x(), 0.0);
}

#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )

// Coordinates utility functions
static inline Vec4d coord(Scalar u = 0, Scalar v = 0)	{ return Vec4d(u, v, 0, 0); }
static inline Vec4d inverseCoord(const Vec4d& c)		{ return Vec4d(1 - c.x(), 1 - c.y(), 0,0); }
static inline std::vector<Vec4d> inverseCoords(const std::vector<Vec4d>& fromCoords)	{ 
	std::vector<Vec4d> invertedCoords;
	foreach(Vec4d coord, fromCoords) invertedCoords.push_back( inverseCoord(coord) );
	return invertedCoords; 
}

static inline double signedAngle(const Vector3 &a, const Vector3 &b, const Vector3 &axis)
{
	if(axis.norm() == 0.0) qDebug() << "warning: zero axis";
	double cosAngle = dot(a.normalized(), b.normalized());
	double angle = acos( qRanged(-1.0, cosAngle, 1.0) );
	Vector3 c = cross(a, b);
	if (dot(c, axis) < 0) return -angle;
	return angle;
}

static inline Vec3d rotatedVec(const Vec3d & v, double theta, const Vec3d & axis)
{
	if(theta == 0.0) return v;
	return (v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)));
}


#define	POINT_ID_RANGE 1000
#define NODE_ID_RANGE	100

static void getIndicesFromSelectedName(int selectedName, int &gID, int &nID, int &pID)
{
	pID = selectedName % POINT_ID_RANGE;
	selectedName /= POINT_ID_RANGE;

	nID = selectedName % NODE_ID_RANGE;
	gID = selectedName / NODE_ID_RANGE;
}

typedef std::pair< QVector<QString>, QVector<QString> > PART_LANDMARK;
typedef std::pair< int, int > POINT_ID;
typedef std::pair< QVector<POINT_ID>, QVector<POINT_ID> > POINT_LANDMARK;

#define AlphaBlend(alpha, start, end) ( ((1-alpha) * start) + (alpha * end) )

static std::vector<Vec3d> refineByNumber(const std::vector<Vec3d> & fromPnts, int targetNumber)
{
	// Refine until targetNumber is achieved
	std::vector<Vec3d> newPnts = fromPnts;
	while(targetNumber > (int)newPnts.size())
	{
		// Find index of largest edge and split
		double maxDist = -DBL_MAX;
		int idx = -1;
		for(int i = 1; i < (int) newPnts.size(); i++){
			double dist = (newPnts[i] - newPnts[i-1]).norm();
			if(dist > maxDist){
				maxDist = dist;
				idx = i;
			}
		}

		Vec3d midPoint = (newPnts[idx] + newPnts[idx-1]) / 2.0;

		// Insert new point
		newPnts.insert( newPnts.begin() + (idx), midPoint );
	}

	return newPnts;
}
