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

// Coordinates
static inline Vec2d inverseCoord(const Vec2d& c)	{ return Vec2d(1 - c.x(), 1 - c.y()); }
static inline Vector3 coord3(const Vec2d& c)		{ return Vector3(c.x(), c.y(), 0); }
static inline Vector3 inverseCoord3(const Vec2d& c) { return coord3(inverseCoord(c)); }
