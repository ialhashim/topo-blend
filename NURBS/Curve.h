// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.

#pragma once

#include "NURBSGlobal.h"

extern int TIME_ITERATIONS;
extern double CURVE_TOLERANCE;
extern int RombergIntegralOrder;

namespace NURBS
{

template <typename Real>
class Curve
{
public:
    // Abstract base class.
    Curve () {}
    Curve (Real tmin, Real tmax);
    virtual ~Curve ();

    // Interval on which curve parameter is defined.  If you are interested
    // in only a subinterval of the actual domain of the curve, you may set
    // that subinterval with SetTimeInterval.  This function requires that
    // tmin < tmax.
    Real GetMinTime () ;
    Real GetMaxTime () ;
    void SetTimeInterval (Real tmin, Real tmax);

    // Position and derivatives.
    virtual Vector3 GetPosition (Real t)  = 0;
    virtual Vector3 GetFirstDerivative (Real t)  = 0;
    virtual Vector3 GetSecondDerivative (Real t)  = 0;
    virtual Vector3 GetThirdDerivative (Real t)  = 0;

    // Differential geometric quantities.
    Real GetSpeed (Real t) ;
    virtual Real GetLength (Real t0, Real t1)  = 0;
    Real GetTotalLength () ;
    Vector3 GetTangent (Real t) ;
    Vector3 GetNormal (Real t) ;
    Vector3 GetBinormal (Real t) ;
    void GetFrame (Real t, Vector3& position, Vector3& tangent, Vector3& normal, Vector3& binormal) ;
    Real GetCurvature (Real t) ;
    Real GetTorsion (Real t) ;

    // Inverse mapping of s = Length(t) given by t = Length^{-1}(s).
    virtual Real GetTime (Real length, int iterations, Real tolerance)  = 0;

    // Subdivision.
    void SubdivideByTime (int numPoints, Array1D_Vector3& points) ;
    void SubdivideByLength (int numPoints, Array1D_Vector3& points) ;

    void SubdivideByLengthTime(int numPoints, std::vector<Real> &times);

    // Curve parameter is t where tmin <= t <= tmax.
    Real mTMin, mTMax;
};

typedef Curve<float> Curvef;
typedef Curve<double> Curved;

}

