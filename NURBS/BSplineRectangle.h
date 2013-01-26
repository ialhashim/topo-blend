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
class BSplineRectangle : public ParametricSurface<Real>
{
public:
    // Construction and destruction.   The caller is responsible for deleting
    // the input arrays if they were dynamically allocated.  Internal copies
    // of the arrays are made, so to dynamically change control points or
    // knots you must use the 'SetControlPoint', 'GetControlPoint', and
    // 'Knot' member functions.

    // Spline types for curves are
    //   open uniform (OU)
    //   periodic uniform (PU)
    //   open nonuniform (ON)
    // For tensor product surfaces, you have to choose a type for each of two
    // dimensions, leading to nine possible spline types for surfaces.  The
    // constructors below represent these choices.

    // (OU,OU), (OU,PU), (PU,OU), or (PU,PU)
    BSplineRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Vector3** ctrlPoint, int uDegree, int vDegree, bool uLoop,
        bool vLoop, bool uOpen, bool vOpen);

    // (OU,ON) or (PU,ON)
    BSplineRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Vector3** ctrlPoint, int uDegree, int vDegree, bool uLoop,
        bool vLoop, bool uOpen, Real* vKnot);

    // (ON,OU) or (ON,PU)
    BSplineRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Vector3** ctrlPoint, int uDegree, int vDegree, bool uLoop,
        bool vLoop, Real* uKnot, bool vOpen);

    // (ON,ON)
    BSplineRectangle (int numUCtrlPoints, int numVCtrlPoints,
        Vector3** ctrlPoint, int uDegree, int vDegree, bool uLoop,
        bool vLoop, Real* uKnot, Real* vKnot);

    int GetNumCtrlPoints (int dim) const;
    int GetDegree (int dim) const;
    bool IsOpen (int dim) const;
    bool IsUniform (int dim) const;
    bool IsLoop (int dim) const;

    // Control points may be changed at any time.  If either input index is
    // invalid, GetControlPoint returns a vector whose components are all
    // MAX_REAL.
    void SetControlPoint (int uIndex, int vIndex, const Vector3& ctrl);
    Vector3 GetControlPoint (int uIndex, int vIndex) const;

    // The knot values can be changed only if the surface is nonuniform in the
    // selected dimension and only if the input index is valid.  If these
    // conditions are not satisfied, GetKnot returns MAX_REAL.
    void SetKnot (int dim, int i, Real knot);
    Real GetKnot (int dim, int i) const;

    // The spline is defined for 0 <= u <= 1 and 0 <= v <= 1.  The input
    // values should be in this domain.  Any inputs smaller than 0 are clamped
    // to 0.  Any inputs larger than 1 are clamped to 1.
    virtual Vector3 P (Real u, Real v) ;
    virtual Vector3 PU (Real u, Real v) ;
    virtual Vector3 PV (Real u, Real v) ;
    virtual Vector3 PUU (Real u, Real v) ;
    virtual Vector3 PUV (Real u, Real v) ;
    virtual Vector3 PVV (Real u, Real v) ;

    // If you need position and derivatives at the same time, it is more
    // efficient to call these functions.  Pass the addresses of those
    // quantities whose values you want.  You may pass 0 in any argument
    // whose value you do not want.
    void Get (Real u, Real v, Vector3* pos, Vector3* derU,
        Vector3* derV, Vector3* derUU, Vector3* derUV,
        Vector3* derVV);

protected:
    // Replicate the necessary number of control points when the Create
    // function has bLoop equal to true, in which case the spline surface
    // must be a closed surface in the corresponding dimension.
    void CreateControl (Vector3** ctrlPoint);

    int mNumUCtrlPoints, mNumVCtrlPoints;
    Array2D_Vector3 mCtrlPoint;  // ctrl[unum][vnum]
    std::vector<bool> mLoop;
    std::vector< BSplineBasis<Real> > mBasis;
    int mUReplicate, mVReplicate;
};

typedef BSplineRectangle<float> BSplineRectanglef;
typedef BSplineRectangle<double> BSplineRectangled;

}
