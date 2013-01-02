#include "Curve.h"
using namespace NURBS;

//----------------------------------------------------------------------------

Curve::Curve (Real tmin, Real tmax)
{
    mTMin = tmin;
    mTMax = tmax;
}
//----------------------------------------------------------------------------

Real Curve::GetMinTime ()
{
    return mTMin;
}
//----------------------------------------------------------------------------

Real Curve::GetMaxTime ()
{
    return mTMax;
}
//----------------------------------------------------------------------------

void Curve::SetTimeInterval (Real tmin, Real tmax)
{
    assert(tmin < tmax);
    mTMin = tmin;
    mTMax = tmax;
}
//----------------------------------------------------------------------------

Real Curve::GetSpeed (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    Real speed = velocity.norm();
    return speed;
}
//----------------------------------------------------------------------------

Real Curve::GetTotalLength ()
{
    return GetLength(mTMin, mTMax);
}
//----------------------------------------------------------------------------

Vector3 Curve::GetTangent (Real t)
{
    Vector3 velocity = GetFirstDerivative(t);
    velocity.normalize();
    return velocity;
}
//----------------------------------------------------------------------------

Vector3 Curve::GetNormal (Real t)
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

Vector3 Curve::GetBinormal (Real t)
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

void Curve::GetFrame (Real t, Vector3& position,
    Vector3& tangent, Vector3& normal, Vector3& binormal)

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

Real Curve::GetCurvature (Real t)
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

Real Curve::GetTorsion (Real t)
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

void Curve::SubdivideByTime (int numPoints, std::vector<Vector3> & points)
{
    assert(numPoints >= 2);
    points.resize(numPoints);

    Real delta = (mTMax - mTMin)/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real t = mTMin + delta*i;
        points[i] = GetPosition(t);
    }
}
//----------------------------------------------------------------------------

void Curve::SubdivideByLength (int numPoints, std::vector<Vector3> & points)
{
    assert(numPoints >= 2);
    points.resize(numPoints);

    Real delta = GetTotalLength()/(numPoints - 1);

    for (int i = 0; i < numPoints; ++i)
    {
        Real length = delta*i;
        Real t = GetTime(length);
        points[i] = GetPosition(t);
    }
}

void Curve::SubdivideByLengthTime (int numPoints, std::vector<Real> & times)
{
	assert(numPoints >= 2);
	times.resize(numPoints);

	Real delta = GetTotalLength()/(numPoints - 1);

	for (int i = 0; i < numPoints; ++i)
	{
		Real length = delta*i;
		Real t = GetTime(length);
		times[i] = t;
	}
}

