// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)

#include "Curve.h"

#ifndef QT_DEBUG
	// Standard resolution
	int TIME_ITERATIONS			= 16;
	double CURVE_TOLERANCE		= 1e-06;
	int RombergIntegralOrder	= 7;
#else
	// Lower resolution
	int TIME_ITERATIONS			= 8;
	double CURVE_TOLERANCE		= 1e-05;
	int RombergIntegralOrder	= 3;
#endif

namespace NURBS
{
//----------------------------------------------------------------------------
template <typename Real>
Curve<Real>::Curve (Real tmin, Real tmax)
{
    mTMin = tmin;
    mTMax = tmax;
}

//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetMinTime ()
{
    return mTMin;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetMaxTime ()
{
    return mTMax;
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve<Real>::SetTimeInterval (Real tmin, Real tmax)
{
    assertion(tmin < tmax, "Invalid time interval\n");
    mTMin = tmin;
    mTMax = tmax;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetSpeed (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Real speed = velocity.norm();
    return speed;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetTotalLength ()
{
    return GetLength(mTMin, mTMax);
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve<Real>::GetTangent (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    velocity.normalize();
    return velocity;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve<Real>::GetNormal (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    Vector3 normal = VDotV*acceleration - VDotA*velocity;
    normal.normalize();
    return normal;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve<Real>::GetBinormal (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    Vector3 normal = VDotV*acceleration - VDotA*velocity;
    normal.normalize();
    velocity.normalize();
    Vector3 binormal = cross(velocity,normal);
    return binormal;
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve<Real>::GetFrame (Real t, Vector3& position,
    Vector3& tangent, Vector3& normal, Vector3& binormal)

{
    position = GetPosition(t);
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    normal = VDotV*acceleration - VDotA*velocity;
    normal.normalize();
    tangent = velocity;
    tangent.normalize();
    binormal = cross(tangent,normal);
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetCurvature (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Real speedSqr = velocity.squaredNorm();

    if (speedSqr >= REAL_ZERO_TOLERANCE)
    {
        Vector3 acceleration = GetSecondDerivative(t);
        Vector3 _cross = cross(velocity,acceleration);
        Real numer = _cross.norm();
        Real denom = pow(speedSqr, (Real)1.5);
        return numer/denom;
    }
    else
    {
        // Curvature is indeterminate, just return 0.
        return (Real)0;
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve<Real>::GetTorsion (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Vector3 _cross = cross(velocity,acceleration);
    Real denom = _cross.squaredNorm();

    if (denom >= REAL_ZERO_TOLERANCE)
    {
        Vector3 jerk = GetThirdDerivative(t);
        Real numer = dot(_cross,jerk);
        return numer/denom;
    }
    else
    {
        // Torsion is indeterminate, just return 0.
        return (Real)0;
    }
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve<Real>::SubdivideByTime (int numPoints, Array1D_Vector3& points)
{
    assertion(numPoints >= 2, "Subdivision requires at least two points\n");
    points = new1<Vector3>(numPoints);

    Real delta = (mTMax - mTMin)/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real t = mTMin + delta*i;
        points[i] = GetPosition(t);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve<Real>::SubdivideByLength (int numPoints, Array1D_Vector3& points)
{
    assertion(numPoints >= 2, "Subdivision requires at least two points\n");
    points = new1<Vector3>(numPoints);

    Real delta = GetTotalLength()/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real length = delta*i;
        Real t = GetTime(length, TIME_ITERATIONS, CURVE_TOLERANCE);
        points[i] = GetPosition(t);
    }
}
//----------------------------------------------------------------------------

template <typename Real>
void Curve<Real>::SubdivideByLengthTime (int numPoints, std::vector<Real> & times)
{
    assert(numPoints >= 2);
    times.resize(numPoints);

    Real delta = GetTotalLength() / (numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real length = delta * i;
        times[i] = GetTime(length, TIME_ITERATIONS, CURVE_TOLERANCE);
    }
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
//template
//class Curve<float>;

template
class Curve<double>;
//----------------------------------------------------------------------------
}
