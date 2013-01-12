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
	// Theta is between Z and <X,Y>, Phi is on <X,Y>
	int thetaResolution, phiResolution;

	// Generate the rays
	QVector<double> result_t;
	QVector< std::pair<double, double> > result_uv;
	QVector<Vec3d> result_rays;
	QVector<double> result_offset;

	void generateRaysFromCurve(NURBSCurve & curve);
	void generateRaysWithinCylinder(NURBSCurve curve);
	void generateRaysWithinHemiSphere(Vector3 X, Vector3 Z);

	void generateRaysFromSheet( NURBSRectangle &sheet );
	void generateRaysOnTwoSidedPlane(NURBSRectangle &sheet);
	void generateRaysOnEdges(NURBSRectangle &sheet);
	void generateRaysWithinSheetCorner(Vector3 X, Vector3 Y, Vector3 Z);

	QVector<Vector3> raysAroundAxis(Vector3 start, Vector3 axis, double range, int resolution);

    // Curves:
    void resampleCurve(Structure::Curve * curve, int timeResolution, int thetaResolution, int phiResolution);

    // Sheets:
    void resampleSheet(Structure::Sheet * sheet, int uResolution, int vResolution, int thetaResolution, int phiResolution);

    /// Blending
    SurfaceMeshModel * blend( Structure::Node * n1, Structure::Node * n2, double t );

    /// Helper functions:
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
