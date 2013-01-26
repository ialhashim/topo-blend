// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.

#pragma once

#include "Surface.h"

namespace NURBS
{

template <typename Real>
class ParametricSurface : public Surface<Real>
{
public:
    // Abstract base class.
    ParametricSurface() {}
    virtual ~ParametricSurface ();

    // The parametric domain is either rectangular or triangular.  Valid (u,v)
    // values for a rectangular domain satisfy
    //   umin <= u <= umax,  vmin <= v <= vmax
    // Valid (u,v) values for a triangular domain satisfy
    //   umin <= u <= umax,  vmin <= v <= vmax,
    //   (vmax-vmin)*(u-umin)+(umax-umin)*(v-vmax) <= 0
    Real GetUMin () const;
    Real GetUMax () const;
    Real GetVMin () const;
    Real GetVMax () const;
    bool IsRectangular () const;

    // position and derivatives up to second order
    virtual Vector3 P (Real u, Real v)  = 0;
    virtual Vector3 PU (Real u, Real v)  = 0;
    virtual Vector3 PV (Real u, Real v)  = 0;
    virtual Vector3 PUU (Real u, Real v)  = 0;
    virtual Vector3 PUV (Real u, Real v)  = 0;
    virtual Vector3 PVV (Real u, Real v)  = 0;

    // Compute a coordinate frame.  The set {T0,T1,N} is a right-handed
    // orthonormal set.
    void GetFrame (Real u, Real v, Vector3& position,
        Vector3& tangent0, Vector3& tangent1,
        Vector3& normal) ;

    // Differential geometric quantities.  The returned scalars are the
    // principal curvatures and the returned vectors are the corresponding
    // principal directions.
    void ComputePrincipalCurvatureInfo (Real u, Real v, Real& curv0,
        Real& curv1, Vector3& dir0, Vector3& dir1);

protected:
    ParametricSurface (Real umin, Real umax, Real vmin, Real vmax,
        bool rectangular);

    Real mUMin, mUMax, mVMin, mVMax;
    bool mRectangular;
};

typedef ParametricSurface<float> ParametricSurfacef;
typedef ParametricSurface<double> ParametricSurfaced;

}
