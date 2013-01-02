#pragma once

#include <assert.h>
#include <QVariant>
#include <QMap>
#include <QSet>

#include "SurfaceMeshHelper.h"
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
	double cosAngle = dot(a.normalized(), b.normalized());
	double angle = acos( qRanged(-1.0, cosAngle, 1.0) );
	Vector3 c = cross(a, b);
	if (dot(c, axis) < 0) return -angle;
	return angle;
}

static inline Vec3d rotatedVec(const Vec3d & v, double theta, const Vec3d & axis)
{
	return (v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)));
}
