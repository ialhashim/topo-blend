#include "ParametricSurface.h"
#include "Matrix2.h"

//----------------------------------------------------------------------------

ParametricSurface::ParametricSurface (Real umin, Real umax, Real vmin, Real vmax, bool rectangular)
{
    assert(umin < umax && vmin < vmax);

    mUMin = umin;
    mUMax = umax;
    mVMin = vmin;
    mVMax = vmax;
    mRectangular = rectangular;
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------

Real ParametricSurface::GetUMin ()
{
    return mUMin;
}
//----------------------------------------------------------------------------

Real ParametricSurface::GetUMax ()
{
    return mUMax;
}
//----------------------------------------------------------------------------

Real ParametricSurface::GetVMin ()
{
    return mVMin;
}
//----------------------------------------------------------------------------

Real ParametricSurface::GetVMax ()
{
    return mVMax;
}
//----------------------------------------------------------------------------

bool ParametricSurface::IsRectangular ()
{
    return mRectangular;
}
//----------------------------------------------------------------------------

void ParametricSurface::GetFrame (Real u, Real v, Vector3& position, Vector3& tangent0, Vector3& tangent1, Vector3& normal)
{
	Vector3 p(0), PU(0), PV(0);

	GetAll(u, v, &p, &PU, &PV);

    position = p;
    tangent0 = PU;
    tangent1 = PV;

    tangent0.normalize();  // T0
    tangent1.normalize();  // temporary T1 just to compute N
    normal = cross(tangent0, tangent1).normalized();  // N

    // The normalized first derivatives are not necessarily orthogonal.
    // Recompute T1 so that {T0,T1,N} is an orthonormal set.
    tangent1 = cross(normal,tangent0);
}
//----------------------------------------------------------------------------

void ParametricSurface::ComputePrincipalCurvatureInfo (Real u, Real v, Real& curv0, Real& curv1, Vector3& dir0, Vector3& dir1)
{
    // Tangents:  T0 = (x_u,y_u,z_u), T1 = (x_v,y_v,z_v)
    // Normal:    N = Cross(T0,T1)/Length(Cross(T0,T1))
    // Metric Tensor:    G = +-                      -+
    //                       | Dot(T0,T0)  Dot(T0,T1) |
    //                       | Dot(T1,T0)  Dot(T1,T1) |
    //                       +-                      -+
    //
    // Curvature Tensor:  B = +-                          -+
    //                        | -Dot(N,T0_u)  -Dot(N,T0_v) |
    //                        | -Dot(N,T1_u)  -Dot(N,T1_v) |
    //                        +-                          -+
    //
    // Principal curvatures k are the generalized eigenvalues of
    //
    //     Bw = kGw
    //
    // If k is a curvature and w=(a,b) is the corresponding solution to
    // Bw = kGw, then the principal direction as a 3D vector is d = a*U+b*V.
    //
    // Let k1 and k2 be the principal curvatures.  The mean curvature
    // is (k1+k2)/2 and the Gaussian curvature is k1*k2.

    // Compute derivatives.
	Vector3 p(0), PU(0), PV(0), PUU(0), PUV(0), PVV(0);
	GetAll(u,v,&p,&PU,&PV,&PUU,&PUV,&PVV);

    Vector3 derU = PU;
    Vector3 derV = PV;
    Vector3 derUU = PUU;
    Vector3 derUV = PUV;
    Vector3 derVV = PVV;

    // Compute the metric tensor.
    Matrix2<Real> metricTensor;
    metricTensor[0][0] = dot(derU,derU);
    metricTensor[0][1] = dot(derU,derV);
    metricTensor[1][0] = metricTensor[0][1];
    metricTensor[1][1] = dot(derV,derV);

    // Compute the curvature tensor.
    Vector3 normal = cross(derU,derV).normalized();
    Matrix2<Real> curvatureTensor;
    curvatureTensor[0][0] = -dot(normal, derUU);
    curvatureTensor[0][1] = -dot(normal, derUV);
    curvatureTensor[1][0] = curvatureTensor[0][1];
    curvatureTensor[1][1] = -dot(normal, derVV);

    // Characteristic polynomial is 0 = det(B-kG) = c2*k^2+c1*k+c0.
    Real c0 = curvatureTensor.Determinant();
    Real c1 = ((Real)2)*curvatureTensor[0][1]* metricTensor[0][1] -
        curvatureTensor[0][0]*metricTensor[1][1] -
        curvatureTensor[1][1]*metricTensor[0][0];
    Real c2 = metricTensor.Determinant();

    // Principal curvatures are roots of characteristic polynomial.
    Real temp = sqrt(abs(c1*c1 - ((Real)4)*c0*c2));
    Real mult = ((Real)0.5)/c2;
    curv0 = -mult*(c1+temp);
    curv1 = mult*(-c1+temp);

    // Principal directions are solutions to (B-kG)w = 0,
    // w1 = (b12-k1*g12,-(b11-k1*g11)) OR (b22-k1*g22,-(b12-k1*g12)).
    Real a0 = curvatureTensor[0][1] - curv0*metricTensor[0][1];
    Real a1 = curv0*metricTensor[0][0] - curvatureTensor[0][0];
    Real length = sqrt(a0*a0 + a1*a1);
    if (length >= REAL_ZERO_TOLERANCE)
    {
        dir0 = a0*derU + a1*derV;
    }
    else
    {
        a0 = curvatureTensor[1][1] - curv0*metricTensor[1][1];
        a1 = curv0*metricTensor[0][1] - curvatureTensor[0][1];
        length = sqrt(a0*a0 + a1*a1);
        if (length >= REAL_ZERO_TOLERANCE)
        {
            dir0 = a0*derU + a1*derV;
        }
        else
        {
            // Umbilic (surface is locally sphere, any direction principal).
            dir0 = derU;
        }
    }
    dir0.normalize();

    // Second tangent is cross product of first tangent and normal.
    dir1 = cross(dir0, normal);
}
//----------------------------------------------------------------------------
