// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)


#include "BSplineCurve.h"

namespace NURBS
{
//----------------------------------------------------------------------------
template <typename Real>
BSplineCurve<Real>::BSplineCurve (int numCtrlPoints,
    const Array1D_Vector3& ctrlPoint, int degree, bool loop, bool open)
    :
    SingleCurve<Real>((Real)0, (Real)1),
    mLoop(loop)
{
    assertion(numCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= degree && degree <= numCtrlPoints-1, "Invalid input\n");

    mNumCtrlPoints = numCtrlPoints;
    mReplicate = (loop ? (open ? 1 : degree) : 0);
    CreateControl(ctrlPoint);
    mBasis.Create(mNumCtrlPoints + mReplicate, degree, open);
}
//----------------------------------------------------------------------------
template <typename Real>
BSplineCurve<Real>::BSplineCurve (int numCtrlPoints,
    const Array1D_Vector3& ctrlPoint, int degree, bool loop,
    const Real* knot)
    :
    SingleCurve<Real>((Real)0, (Real)1),
    mLoop(loop)
{
    assertion(numCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= degree && degree <= numCtrlPoints-1, "Invalid input\n");

    mNumCtrlPoints = numCtrlPoints;
    mReplicate = (loop ? 1 : 0);
    CreateControl(ctrlPoint);
    mBasis.Create(mNumCtrlPoints + mReplicate, degree, knot);
}
//----------------------------------------------------------------------------
template <typename Real>
void BSplineCurve<Real>::CreateControl (const Array1D_Vector3& ctrlPoint)
{
    int newNumCtrlPoints = mNumCtrlPoints + mReplicate;
    newNumCtrlPoints = newNumCtrlPoints;

    mCtrlPoint = ctrlPoint;

    for (int i = 0; i < mReplicate; ++i)
    {
        mCtrlPoint[mNumCtrlPoints + i] = ctrlPoint[i];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
int BSplineCurve<Real>::GetNumCtrlPoints () const
{
    return mNumCtrlPoints;
}
//----------------------------------------------------------------------------
template <typename Real>
int BSplineCurve<Real>::GetDegree () const
{
    return mBasis.GetDegree();
}
//----------------------------------------------------------------------------
template <typename Real>
bool BSplineCurve<Real>::IsOpen () const
{
    return mBasis.IsOpen();
}
//----------------------------------------------------------------------------
template <typename Real>
bool BSplineCurve<Real>::IsUniform () const
{
    return mBasis.IsUniform();
}
//----------------------------------------------------------------------------
template <typename Real>
bool BSplineCurve<Real>::IsLoop () const
{
    return mLoop;
}
//----------------------------------------------------------------------------
template <typename Real>
void BSplineCurve<Real>::SetControlPoint (int i, const Vector3& ctrl)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        // Set the control point.
        mCtrlPoint[i] = ctrl;

        // Set the replicated control point.
        if (i < mReplicate)
        {
            mCtrlPoint[mNumCtrlPoints + i] = ctrl;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 BSplineCurve<Real>::GetControlPoint (int i) const
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlPoint[i];
    }

    return Vector3(std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max(),
        std::numeric_limits<Real>::max());
}
//----------------------------------------------------------------------------
template <typename Real>
void BSplineCurve<Real>::SetKnot (int i, Real knot)
{
    mBasis.SetKnot(i, knot);
}
//----------------------------------------------------------------------------
template <typename Real>
Real BSplineCurve<Real>::GetKnot (int i) const
{
    return mBasis.GetKnot(i);
}
//----------------------------------------------------------------------------
template <typename Real>
void BSplineCurve<Real>::Get (Real t, Vector3* pos,
    Vector3* der1, Vector3* der2, Vector3* der3)
{
    int i, imin, imax;
    if (der3)
    {
        mBasis.Compute(t, 0, imin, imax);
        mBasis.Compute(t, 1, imin, imax);
        mBasis.Compute(t, 2, imin, imax);
        mBasis.Compute(t, 3, imin, imax);
    }
    else if (der2)
    {
        mBasis.Compute(t, 0, imin, imax);
        mBasis.Compute(t, 1, imin, imax);
        mBasis.Compute(t, 2, imin, imax);
    }
    else if (der1)
    {
        mBasis.Compute(t, 0, imin, imax);
        mBasis.Compute(t, 1, imin, imax);
    }
    else
    {
        mBasis.Compute(t, 0, imin, imax);
    }

    if (pos)
    {
        *pos = Vector3_ZERO;
        for (i = imin; i <= imax; ++i)
        {
            *pos += mBasis.GetD0(i)*mCtrlPoint[i];
        }
    }

    if (der1)
    {
        *der1 = Vector3_ZERO;
        for (i = imin; i <= imax; ++i)
        {
            *der1 += mBasis.GetD1(i)*mCtrlPoint[i];
        }
    }

    if (der2)
    {
        *der2 = Vector3_ZERO;
        for (i = imin; i <= imax; ++i)
        {
            *der2 += mBasis.GetD2(i)*mCtrlPoint[i];
        }
    }

    if (der3)
    {
        *der3 = Vector3_ZERO;
        for (i = imin; i <= imax; ++i)
        {
            *der3 += mBasis.GetD3(i)*mCtrlPoint[i];
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
BSplineBasis<Real>& BSplineCurve<Real>::GetBasis ()
{
    return mBasis;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 BSplineCurve<Real>::GetPosition (Real t)
{
    Vector3 pos;
    Get(t, &pos, 0, 0, 0);
    return pos;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 BSplineCurve<Real>::GetFirstDerivative (Real t)
{
    Vector3 der1;
    Get(t, 0, &der1, 0, 0);
    return der1;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 BSplineCurve<Real>::GetSecondDerivative (Real t)
{
    Vector3 der2;
    Get(t, 0, 0, &der2, 0);
    return der2;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 BSplineCurve<Real>::GetThirdDerivative (Real t)
{
    Vector3 der3;
    Get(t, 0, 0, 0, &der3);
    return der3;
}
//----------------------------------------------------------------------------

template <typename Real>
Array1D_Real NURBS::BSplineCurve<Real>::GetKnotVector(bool isInnerOnly)
{
	if(isInnerOnly)
	{
		int d = this->GetDegree() + 1;
		
		Array1D_Real result(mBasis.mKnot.begin() + d, mBasis.mKnot.end() - d);
		return result;
	}
	else
		return this->mBasis.mKnot;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
//template
//class BSplineCurve<float>;

template
class BSplineCurve<double>;
//----------------------------------------------------------------------------
}
