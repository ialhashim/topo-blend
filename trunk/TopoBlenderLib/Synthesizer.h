#pragma once

#include "StructureGraph.h"
using namespace Structure;

#include "Octree.h"
Q_DECLARE_METATYPE(Octree *)

struct Sample{
    double u,v;
    Vec3d d;
    Sample(const Vec3d & D, double U, double V = 0) { u = U; v = V; d = D; }
};

struct Synthesizer
{
    Synthesizer();

	// Parameters
	int timeResolution;
	int uResolution, vResolution;
	int thetaResolution, phiResolution;

	// Generate the rays
	QVector<double> result_t;
	QVector<double> result_u, result_v;
	QVector<Vec3d> result_rays;
	QVector<double> result_offset;
	void generateRaysFromCurve(NURBSCurve & curve);
	void generateRaysFromSheet(Structure::Sheet * sheet);

	void generateCylindricalRays(NURBSCurve curve, double thetaRange);
	void generateSphericalRays(NURBSCurve curve, double t, double thetaRange, double phiRange);

	// Legency 
    /// Resampling
    // Curves:
    void resampleCurve(Structure::Curve * curve, int timeResolution, int thetaResolution, int phiResolution);

    // Sheets:
    void resampleSheet(Structure::Sheet * sheet, int uResolution, int vResolution, int thetaResolution, int phiResolution);

    /// Blending
    SurfaceMeshModel * blend( Structure::Node * n1, Structure::Node * n2, double t );

    /// Helper functions:
    Array2D_Vector3 cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection, int timeResolution, int thetaResolution, double thetaRange, Octree *octree, bool SheetBoundary);
    Array2D_Vector3 sphereResampling(Array2D_Vector3 mesh_faces, Vector3 endPoint,Vector3 thetaAxis, Vector3 phiAxis, int thetaResolution,int phiResolution, double thetaRange, double phiRange, Octree *octree);
    Array1D_Vector3 resampleCoutourPointsPlane(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,double curr_t, int thetaResolution, double thetaRange, Vector3 fixed_startDirection, Octree *octree);
    std::vector<Array2D_Vector3> planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, Vector3 initialDirection, int uResolution, int vResolution, Octree *octree);

    void addEndFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel *mesh, std::vector<SurfaceMeshModel::Vertex> &vertices_idx, int idxBase);
    void addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel *mesh, std::vector<Vertex> &vertices_idx, int idxBase);
    void addCornerFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel *mesh, std::vector<Vertex> &vertices_idx, int idxBase);
    void addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane, SurfaceMeshModel *mesh, std::vector<Vertex> &vertices_idx, int idxBase);

    Vec3d intersectionPoint(Ray ray, Octree *useTree, int *faceIndex = NULL);
};

// Helper macros:
#define SIGN(x) (((x) < 0) ? (-1) : (((x) > 0) ? 1 : 0))
#define ROTATED_VEC(v, theta, axis) (v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)))
#define VEC_FROM_POINTS(a,b,c) \
    (a)[0] = (b)[0] - (c)[0];	\
    (a)[1] = (b)[1] - (c)[1];	\
    (a)[2] = (b)[2] - (c)[2];
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
