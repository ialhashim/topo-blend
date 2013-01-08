#pragma once
#include <float.h>
#include "SurfaceMeshModel.h"
#include "SurfaceMeshHelper.h"
using namespace SurfaceMeshTypes;

namespace VoxelerLibrary{

struct Voxel{ 
	int x, y, z;
	int flag;

	Voxel(){ x = y = z = flag = 0; }
	Voxel(int X, int Y, int Z) : x(X), y(Y), z(Z){ flag = 1; } 

	// Operators
    operator const Vector3() const{ return Vector3(x,y,z); }

	Voxel & operator+= (const Voxel & other){
		x += other.x;	y += other.y;	z += other.z;
		return *this;
	}

	Voxel operator+(const Voxel & other) const{
		return Voxel(*this) += other;
	}

	bool operator== (const Voxel & other) const{
		return this->x == other.x && this->y == other.y && this->z == other.z;
	}

	// Useful for bounds
    inline void toMax(const Voxel & v){ x = qMax(x, v.x); y = qMax(y, v.y); z = qMax(z, v.z); }
    inline void toMin(const Voxel & v){ x = qMin(x, v.x); y = qMin(y, v.y); z = qMin(z, v.z); }
};

struct FaceBounds { int minX, minY, minZ; int maxX, maxY, maxZ; };

}
