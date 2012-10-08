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
    std::vector<Voxel> voxelTorus(double pathRadius, double circleRadius);
    std::vector<Voxel> voxelLine(const Vec3d & p1, const Vec3d & p2, bool thick = false);
    std::vector<Voxel> voxelCircle(double radius);
    std::vector<Voxel> orientedVoxelCircle(double radius, const Vec3d &direction);

    void addLine(const Vec3d & p1, const Vec3d &p2);
    void addSphere(const Vec3d & center, double radius);
    void addHemiSphere(const Vec3d & center, double radius, const Vec3d &direction=Vec3d(0,0,1));
    void addCircle(const Vec3d & center, double radius, const Vec3d &direction=Vec3d(0,0,1));
    void addCylinder(const Vec3d & from, const Vec3d & to, double radius);
    void addCapsule(const Vec3d & from, const Vec3d & to, double radius);
    void addPolyLine(const QVector<Vec3d> &points, double radius);
    void addTorus(const Vec3d & center, double pathRadius, double circleRadius, const Vec3d &direction=Vec3d(0,0,1));

    void begin();
    void setVoxel(int x, int y, int z);
    void end();

    QVector<Voxel> toProcess;
    QSet<Voxel> voxels;

    Voxel minVoxel, maxVoxel;

    double voxel_size;

    void buildMesh(SurfaceMeshModel *m);
    void MeanCurvatureFlow( SurfaceMeshModel *m );
    void LaplacianSmoothing( SurfaceMeshModel *m, bool protectBorders = false );

};

