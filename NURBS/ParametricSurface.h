#pragma once

#include "Curve.h"

class ParametricSurface
{
public:

    ParametricSurface(){}

    // The parametric domain is either rectangular or triangular.  Valid (u,v)
    // values for a rectangular domain satisfy
    //   umin <= u <= umax,  vmin <= v <= vmax
    // Valid (u,v) values for a triangular domain satisfy
    //   umin <= u <= umax,  vmin <= v <= vmax,
    //   (vmax-vmin)*(u-umin)+(umax-umin)*(v-vmax) <= 0
    Real GetUMin();
    Real GetUMax();
    Real GetVMin();
    Real GetVMax();
    bool IsRectangular();

    // Compute a coordinate frame.  The set {T0,T1,N} is a right-handed
    // orthonormal set.
    void GetFrame (Real u, Real v, Vector3& position, Vector3& tangent0, Vector3& tangent1, Vector3& normal) ;
	virtual void GetAll (Real u, Real v, Vector3 * pos, Vector3* derU = 0, Vector3* derV = 0, Vector3* derUU = 0, Vector3* derUV = 0, Vector3* derVV = 0) = 0;

    // Differential geometric quantities.  The returned scalars are the
    // principal curvatures and the returned vectors are the corresponding
    // principal directions.
    void ComputePrincipalCurvatureInfo (Real u, Real v, Real& curv0, Real& curv1, Vector3& dir0, Vector3& dir1);

protected:
    ParametricSurface (Real umin, Real umax, Real vmin, Real vmax, bool rectangular);

    Real mUMin, mUMax, mVMin, mVMax;
    bool mRectangular;
};

