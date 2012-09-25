#pragma once

#include "SurfaceMeshHelper.h"

class Plane {
public:
	Plane(const Vec3d & plane_center, const Vec3d & plane_normal)
	{
		this->center = plane_center;
		this->n = plane_normal;
	}

	Vec3d center, n;
};