#pragma once

#include "SurfaceMeshModel.h"

#include "Voxel.h"
#include "../CustomDrawObjects.h"

namespace DynamicVoxelLib{

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
	void addBox(const Vec3d & minimum, const Vec3d & maximum);

    void begin();
    void setVoxel(int x, int y, int z);
    void end();

    std::vector<Voxel> voxels;

	double voxel_size;
    Voxel minVoxel, maxVoxel;

	struct QuadMesh{
        std::vector<SurfaceMesh::Vector3> points, debug;
		std::vector<QuadFace> faces;
		void clear() { points.clear(); faces.clear(); }
	};

	QuadMesh toQuadMesh();

    void buildMesh( SurfaceMesh::Model * mesh );
    void buildMesh( SurfaceMesh::Model * mesh, QuadMesh & m );

	// Mesh smoothing
    static void MeanCurvatureFlow( SurfaceMesh::Model *m, double dt = 0.5 );
    static void LaplacianSmoothing( SurfaceMesh::Model *m, bool protectBorders = false );

	// Basic hole filling
    static void FillHoles( SurfaceMesh::Model *m, double voxel_length );
    static bool hasHoles( SurfaceMesh::Model *m );
};

}
