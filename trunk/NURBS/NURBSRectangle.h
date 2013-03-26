// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.

#pragma once

#include "NURBSGlobal.h"
#include "ParametricSurface.h"
#include "BSplineBasis.h"

namespace NURBS
{

template <typename Real>
class NURBSRectangle : public ParametricSurface<Real>
{
public:
    NURBSRectangle() {}

    // Construction and destruction.   The caller is responsible for deleting
    // the input arrays if they were dynamically allocated.  Internal copies
    // of the arrays are made, so to dynamically change control points,
    // control weights, or knots, you must use the 'SetControlPoint',
    // 'GetControlPoint', 'SetControlWeight', 'GetControlWeight', and 'Knot'
    // member functions.

    // The homogeneous input points are (x,y,z,w) where the (x,y,z) values are
    // stored in the akCtrlPoint array and the w values are stored in the
    // afCtrlWeight array.  The output points from curve evaluations are of
    // the form (x',y',z') = (x/w,y/w,z/w).

    // Spline types for curves are
    //   open uniform (OU)
    //   periodic uniform (PU)
    //   open nonuniform (ON)
    // For tensor product surfaces, you have to choose a type for each of two
    // dimensions, leading to nine possible spline types for surfaces.  The
    // constructors below represent these choices.

    // (OU,OU), (OU,PU), (PU,OU), or (PU,PU)
    NURBSRectangle (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight, int uDegree,
        int vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);

    // (OU,ON) or (PU,ON)
    NURBSRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight, int uDegree,
        int vDegree, bool uLoop, bool vLoop, bool uOpen, Real* vKnot);

    // (ON,OU) or (ON,PU)
    NURBSRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight, int uDegree,
        int vDegree, bool uLoop, bool vLoop, Real* uKnot, bool vOpen);

    // (ON,ON)
    NURBSRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight, int uDegree,
        int vDegree, bool uLoop, bool vLoop, Real* uKnot, Real* vKnot);

    static NURBSRectangle<Real> createSheet(Scalar width, Scalar length, Vector3 center, Vector3 dU, Vector3 dV, int nU = 5, int nV = 5);
    static NURBSRectangle<Real> createSheet(Vec3d corner1, Vec3d corner2, int stepsU = 5, int stepsV = 5);
	static NURBSRectangle<Real> createSheetFromPoints( Array2D_Vector3 ctrlPoint );

    int GetNumCtrlPoints (int dim) const;
    int GetDegree (int dim) const;
    bool IsOpen (int dim) const;
    bool IsUniform (int dim) const;
    bool IsLoop (int dim) const;

    // Control points and weights may be changed at any time.  If either input
    // index is invalid, GetControlPoint returns a vector whose components
    // are all MAX_REAL, and GetControlWeight returns MAX_REAL.
    void SetControlPoint (int uIndex, int vIndex, const Vector3& ctrl);
    Vector3 GetControlPoint (int uIndex, int vIndex) const;
    void SetControlWeight (int uIndex, int vIndex, Real weight);
    Real GetControlWeight (int uIndex, int vIndex) const;

    // The knot values can be changed only if the surface is nonuniform in the
    // selected dimension and only if the input index is valid.  If these
    // conditions are not satisfied, GetKnot returns MAX_REAL.
    void SetKnot (int dim, int i, Real knot);
    Real GetKnot (int dim, int i) const;

	void refineU(Array1D_Real & insknts, Array2D_Vector3 & Qw);
	void refine(Array1D_Real & insknts, Array2D_Vector3 & Qw, int dir);
	Array2D_Vector3 midPointRefined();

	static Array2D_Vector3 swapUV( const Array2D_Vector3 & controlPoints );

	Array1D_Real GetKnotVectorU(bool isInnerOnly = false);
	Array1D_Real GetKnotVectorV(bool isInnerOnly = false);

	Array2D_Vector3 simpleRefine( int k = 1, int dir = 0 );
	Array2D_Vector3 simpleRemove( int idx, int dir = 0 );

    // The spline is defined for 0 <= u <= 1 and 0 <= v <= 1.  The input
    // values should be in this domain.  Any inputs smaller than 0 are clamped
    // to 0.  Any inputs larger than 1 are clamped to 1.
    virtual Vector3 P (Real u, Real v);
    virtual Vector3 PU (Real u, Real v);
    virtual Vector3 PV (Real u, Real v);
    virtual Vector3 PUU (Real u, Real v);
    virtual Vector3 PUV (Real u, Real v);
    virtual Vector3 PVV (Real u, Real v);

    // If you need position and derivatives at the same time, it is more
    // efficient to call these functions.  Pass the addresses of those
    // quantities whose values you want.  You may pass 0 in any argument
    // whose value you do not want.
    void Get (Real u, Real v, Vector3* pos, Vector3* derU = 0,
        Vector3* derV = 0, Vector3* derUU = 0, Vector3* derUV = 0,
        Vector3* derVV = 0);

    // Cached visualization
    std::vector<SurfaceQuad> quads;
    void generateSurfaceQuads( double resolution );

    std::vector<std::vector<Vector3> > generateSurfaceTris(Scalar resolution);
    std::vector<std::vector<Vector3> > triangulateControlCage();

    void uniformCoordinates(std::vector<Real> &valU, std::vector<Real> &valV, double resolution, int u = 0, int v = 0);

    void generateSurfacePoints(Scalar stepSize, std::vector<std::vector<Vector3> > &points, std::vector<Real> &valU, std::vector<Real> &valV);
    void generateSurfacePointsCoords(Scalar stepSize, std::vector<std::vector<Vec4d> > &points);

    std::vector<Vector3> GetControlPointsV(int vIndex);
    std::vector<Vector3> GetControlPointsU(int uIndex);

    std::vector<Vec3d> intersect(NURBSRectangle<Real> &other, double resolution, std::vector<Vec4d> &coordMe, std::vector<Vec4d> &coordOther);

    Vec4d timeAt(const Vector3 &pos);
    Vec4d timeAt(const Vector3 &pos, Vec4d &bestUV, Vec4d &minRange, Vec4d &maxRange, Real currentDist, Real threshold = 1e-4 );
    std::vector<Vec4d> timeAt(const std::vector<Vector3> &positions, Real threshold);

    Vector3 projectOnControl(Real u, Real v);

    void translate(const Vec3d &delta);
    void scale(Scalar scaleFactor);

public:
    // Replicate the necessary number of control points when the Create
    // function has bLoop equal to true, in which case the spline surface
    // must be a closed surface in the corresponding dimension.
    void CreateControl (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight);

    int mNumUCtrlPoints, mNumVCtrlPoints;
    Array2D_Vector3 mCtrlPoint;  //   ctrl[unum][vnum]
    Array2D_Real mCtrlWeight;    // weight[unum][vnum]
    std::vector<bool> mLoop;
    std::vector< BSplineBasis<Real> > mBasis;
    int mUReplicate, mVReplicate;

	// Misc
	Array1D_Vector3 misc_points;
};

typedef NURBSRectangle<float> NURBSRectanglef;
typedef NURBSRectangle<double> NURBSRectangled;

}
