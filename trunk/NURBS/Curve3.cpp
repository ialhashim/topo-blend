#include "Curve3.h"

//----------------------------------------------------------------------------
template <typename Real>
Curve3<Real>::Curve3 (Real tmin, Real tmax)
{
    mTMin = tmin;
    mTMax = tmax;
}
//----------------------------------------------------------------------------
template <typename Real>
Curve3<Real>::~Curve3 ()
{
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve3<Real>::GetMinTime () const
{
    return mTMin;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve3<Real>::GetMaxTime () const
{
    return mTMax;
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve3<Real>::SetTimeInterval (Real tmin, Real tmax)
{
    assert(tmin < tmax);
    mTMin = tmin;
    mTMax = tmax;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve3<Real>::GetSpeed (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    Real speed = velocity.norm();
    return speed;
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve3<Real>::GetTotalLength () const
{
    return GetLength(mTMin, mTMax);
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve3<Real>::GetTangent (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    velocity.normalize();
    return velocity;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve3<Real>::GetNormal (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    Vector3 normal = acceleration*VDotV - velocity*VDotA;
    normal.normalize();
    return normal;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 Curve3<Real>::GetBinormal (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    Vector3 normal = acceleration*VDotV - velocity*VDotA;
    normal.normalize();
    velocity.normalize();
    Vector3 binormal = cross(velocity,normal);
    return binormal;
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve3<Real>::GetFrame (Real t, Vector3& position,
    Vector3& tangent, Vector3& normal, Vector3& binormal)
    const
{
    position = GetPosition(t);
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Real VDotV = dot(velocity,velocity);
    Real VDotA = dot(velocity,acceleration);
    normal = acceleration*VDotV - velocity*VDotA;
    normal.normalize();
    tangent = velocity;
    tangent.normalize();
    binormal = cross(tangent,normal);
}
//----------------------------------------------------------------------------
template <typename Real>
Real Curve3<Real>::GetCurvature (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    Real speedSqr = velocity.sqrnorm();

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
Real Curve3<Real>::GetTorsion (Real t) const
{
    Vector3 velocity = GetFirstDerivative(t);
    Vector3 acceleration = GetSecondDerivative(t);
    Vector3 _cross = cross(velocity,acceleration);
    Real denom = _cross.sqrnorm();

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
void Curve3<Real>::SubdivideByTime (int numPoints,
    Vector3*& points) const
{
    assert(numPoints >= 2);
    points = new Vector3[numPoints];

    Real delta = (mTMax - mTMin)/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real t = mTMin + delta*i;
        points[i] = GetPosition(t);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
void Curve3<Real>::SubdivideByLength (int numPoints,
    Vector3*& points) const
{
    assert(numPoints >= 2);
    points = new Vector3[numPoints];

    Real delta = GetTotalLength()/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real length = delta*i;
        Real t = GetTime(length);
        points[i] = GetPosition(t);
    }
}
//----------------------------------------------------------------------------
//
////----------------------------------------------------------------------------
//// Explicit instantiation.
////----------------------------------------------------------------------------
template
class Curve3<float>;

template
class Curve3<double>;
//----------------------------------------------------------------------------
