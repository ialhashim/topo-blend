#pragma once

#include "NanoKdTree.h"
#include "Voxel.h"

#include "SurfaceMeshModel.h"
#include "../segment/CustomDrawObjects.h"

class DynamicVoxel
{
public:
    DynamicVoxel(double voxelSize = 0.1);

    void draw();

    std::vector<Voxel> voxelSphere(double radius);
    std::vector<Voxel> voxelLine(const Vec3d & p1, const Vec3d & p2, bool thick = false);
    std::vector<Voxel> voxelCircle(double radius);
    std::vector<Voxel> orientedVoxelCircle(double radius, const Vec3d &direction);

    void addLine(const Vec3d & p1, const Vec3d &p2);
    void addSphere(const Vec3d & center, double radius);
    void addHemiSphere(const Vec3d & center, double radius, const Vec3d &direction=Vec3d(0,0,1));
    void addCircle(const Vec3d & center, double radius, const Vec3d &direction=Vec3d(0,0,1));
    void addCylinder(const Vec3d & from, const Vec3d & to, double radius);
    void addCapsule(const Vec3d & from, const Vec3d & to, double radius);

    void setVoxel(int x, int y, int z);

    QSet<Voxel> voxels;

    Voxel minVoxel, maxVoxel;

    double voxel_size;

    void buildMesh(SurfaceMeshModel *m);
};

// 3D vector rotation
#define RANGED(min, v, max) ( qMax(min, qMin(v, max)) )
#define ROTATE_VEC(v, theta, axis) (v = v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)))
template <typename VECTYPE>
void inline RotateFromTo(VECTYPE from, VECTYPE to, VECTYPE & point)
{
    if(from.x() == to.x() && from.y() == to.y()) return;
    VECTYPE axis = cross(from, to).normalized();
    double theta = acos( RANGED(-1.0, dot(from.normalize(), to.normalize()), 1.0) );
    ROTATE_VEC(point, theta, axis);
}
