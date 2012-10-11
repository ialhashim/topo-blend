#pragma once

#include <assert.h>
#include <QVariant>
#include <QMap>
#include <QSet>

#include "SurfaceMeshHelper.h"
#include "NurbsDraw.h"
#include "../CustomDrawObjects.h"

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

#undef max
