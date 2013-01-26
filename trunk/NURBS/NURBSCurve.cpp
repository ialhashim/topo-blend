// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)


#include "NURBSCurve.h"
#include "LineSegment.h"

namespace NURBS
{
//----------------------------------------------------------------------------
template <typename Real>
NURBSCurve<Real>::NURBSCurve (
    const Array1D_Vector3 & ctrlPoint, const Array1D_Real & ctrlWeight, int degree,
    bool loop, bool open)
    :
    SingleCurve<Real>((Real)0, (Real)1),
    mLoop(loop)
{
    int numCtrlPoints = ctrlPoint.size();

    assertion(numCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= degree && degree <= numCtrlPoints-1, "Invalid input\n");

    mNumCtrlPoints = numCtrlPoints;
    mReplicate = (loop ? (open ? 1 : degree) : 0);
    CreateControl(ctrlPoint, ctrlWeight);
    mBasis.Create(mNumCtrlPoints + mReplicate, degree, open);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSCurve<Real>::NURBSCurve (int numCtrlPoints,
    const Array1D_Vector3 & ctrlPoint, const Array1D_Real & ctrlWeight, int degree,
    bool loop, const Real* knot)
    :
    SingleCurve<Real>((Real)0, (Real)1),
    mLoop(loop)
{
    assertion(numCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= degree && degree <= numCtrlPoints-1, "Invalid input\n");

    mNumCtrlPoints = numCtrlPoints;
    mReplicate = (loop ? 1 : 0);
    CreateControl(ctrlPoint, ctrlWeight);
    mBasis.Create(mNumCtrlPoints + mReplicate, degree, knot);
}

//----------------------------------------------------------------------------
template <typename Real>
NURBSCurve<Real> NURBSCurve<Real>::createCurve( Vector3 from, Vector3 to, int steps )
{
    std::vector<Vector3> ctrlPoint;

    int degree = 3;

    Vector3 delta = (to - from) / steps;

    for(int i = 0; i <= steps; i++)
        ctrlPoint.push_back(from + (delta * i));

    std::vector<Real> ctrlWeight(ctrlPoint.size(), 1.0);
    return NURBSCurve(ctrlPoint, ctrlWeight, degree, false, true);
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::CreateControl (const Array1D_Vector3 & ctrlPoint, const Array1D_Real & ctrlWeight)
{
    int newNumCtrlPoints = mNumCtrlPoints + mReplicate;
    newNumCtrlPoints = newNumCtrlPoints;

    mCtrlPoint = ctrlPoint;

    mCtrlWeight = ctrlWeight;

    for (int i = 0; i < mReplicate; ++i)
    {
        mCtrlPoint[mNumCtrlPoints+i] = ctrlPoint[i];
        mCtrlWeight[mNumCtrlPoints+i] = ctrlWeight[i];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
int NURBSCurve<Real>::GetNumCtrlPoints () const
{
    return mNumCtrlPoints;
}
//----------------------------------------------------------------------------
template <typename Real>
int NURBSCurve<Real>::GetDegree () const
{
    return mBasis.GetDegree();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsOpen () const
{
    return mBasis.IsOpen();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsUniform () const
{
    return mBasis.IsUniform();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsLoop () const
{
    return mLoop;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::SetControlPoint (int i, const Vector3& ctrl)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        // Set the control point.
        mCtrlPoint[i] = ctrl;

        // Set the replicated control point.
        if (i < mReplicate)
        {
            mCtrlPoint[mNumCtrlPoints+i] = ctrl;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSCurve<Real>::GetControlPoint (int i) const
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
void NURBSCurve<Real>::SetControlWeight (int i, Real weight)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        // Set the control weight.
        mCtrlWeight[i] = weight;

        // Set the replicated control weight.
        if (i < mReplicate)
        {
            mCtrlWeight[mNumCtrlPoints+i] = weight;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Real NURBSCurve<Real>::GetControlWeight (int i) const
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlWeight[i];
    }

    return std::numeric_limits<Real>::max();
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::SetKnot (int i, Real knot)
{
    mBasis.SetKnot(i, knot);
}
//----------------------------------------------------------------------------
template <typename Real>
Real NURBSCurve<Real>::GetKnot (int i) const
{
    return mBasis.GetKnot(i);
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::Get (Real t, Vector3* pos,
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
    else  // pos
    {
        mBasis.Compute(t, 0, imin, imax);
    }

    Real tmp;

    // Compute position.
    Vector3 X = Vector3_ZERO;
    Real w = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD0(i)*mCtrlWeight[i];
        X += tmp*mCtrlPoint[i];
        w += tmp;
    }
    Real invW = ((Real)1)/w;
    Vector3 P = invW*X;
    if (pos)
    {
        *pos = P;
    }

    if (!der1 && !der2 && !der3)
    {
        return;
    }

    // Compute first derivative.
    Vector3 XDer1 = Vector3_ZERO;
    Real wDer1 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD1(i)*mCtrlWeight[i];
        XDer1 += tmp*mCtrlPoint[i];
        wDer1 += tmp;
    }
    Vector3 PDer1 = invW*(XDer1 - wDer1*P);
    if (der1)
    {
        *der1 = PDer1;
    }

    if (!der2 && !der3)
    {
        return;
    }

    // Compute second derivative.
    Vector3 XDer2 = Vector3_ZERO;
    Real wDer2 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD2(i)*mCtrlWeight[i];
        XDer2 += tmp*mCtrlPoint[i];
        wDer2 += tmp;
    }
    Vector3 PDer2 = invW*(XDer2 - ((Real)2)*wDer1*PDer1 - wDer2*P);
    if (der2)
    {
        *der2 = PDer2;
    }

    if (!der3)
    {
        return;
    }

    // Compute third derivative.
    Vector3 XDer3 = Vector3_ZERO;
    Real wDer3 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD3(i)*mCtrlWeight[i];
        XDer3 += tmp*mCtrlPoint[i];
        wDer3 += tmp;
    }
    if (der3)
    {
        *der3 = invW*(XDer3 - ((Real)3)*wDer1*PDer2 -
            ((Real)3)*wDer2*PDer1 - wDer3*P);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
BSplineBasis<Real>& NURBSCurve<Real>::GetBasis ()
{
    return mBasis;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSCurve<Real>::GetPosition (Real t)
{
    Vector3 pos;
    Get(t, &pos, 0, 0, 0);
    return pos;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSCurve<Real>::GetFirstDerivative (Real t)
{
    Vector3 der1;
    Get(t, 0, &der1, 0, 0);
    return der1;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSCurve<Real>::GetSecondDerivative (Real t)
{
    Vector3 der2;
    Get(t, 0, 0, &der2, 0);
    return der2;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSCurve<Real>::GetThirdDerivative (Real t)
{
    Vector3 der3;
    Get(t, 0, 0, 0, &der3);
    return der3;
}
//----------------------------------------------------------------------------

std::vector<Vector3> NURBSCurve<Real>::getControlPoints()
{
    return mCtrlPoint;
}

template <typename Real>
Real NURBSCurve<Real>::timeAt( const Vector3 & pos )
{
    std::vector<Vector3> curvePoints;

    double timeStep = 0.1;

    SubdivideByLength(1 + (1.0 / timeStep), curvePoints);

    int minIdx = 0;
    double t = 0.0;
    Vector3 d(0);
    Scalar minDist = std::numeric_limits<Scalar>::max();

    for(int i = 0; i < (int)curvePoints.size() - 1; i++)
    {
        Line segment(curvePoints[i], curvePoints[i + 1]);
        segment.ClosestPoint(pos, t, d);

        Scalar dist = (pos - d).norm();

        if(dist < minDist){
            minDist = dist;
            minIdx = i;
        }
    }

    Line closestSegment(curvePoints[minIdx], curvePoints[minIdx + 1]);
    closestSegment.ClosestPoint(pos, t, d);
    Scalar found_t = qMax(0.0, qMin( (timeStep * minIdx) + (t * timeStep), 1.0) );
    return found_t;
}

template <typename Real>
void NURBSCurve<Real>::translate( const Vector3 & delta )
{
    for(int i = 0; i < (int)mCtrlPoint.size(); i++)
        mCtrlPoint[i] += delta;
}

template <typename Real>
void NURBSCurve<Real>::scale( Scalar scaleFactor )
{
    for(int i = 0; i < (int)mCtrlPoint.size(); i++)
        mCtrlPoint[i] *= scaleFactor;
}

template <typename Real>
void NURBSCurve<Real>::scaleInPlace( Scalar scaleFactor, int placeCtrlPoint )
{
    Vector3 delta = mCtrlPoint[placeCtrlPoint];
    translate( -delta );
    scale(scaleFactor);
    translate( delta );
}

template <typename Real>
void NURBSCurve<Real>::translateTo( const Vector3 & newPos, int cpIDX )
{
    Vec3d cp = mCtrlPoint[cpIDX];
    Vec3d delta = newPos - cp;
    for(int i = 0; i < GetNumCtrlPoints(); i++)
        mCtrlPoint[i] += delta;
}

template <typename Real>
std::vector < std::vector<Vector3> > NURBSCurve<Real>::toSegments( Scalar resolution )
{
    std::vector< std::vector<Vector3> > segments;


    double firstTwoDist = (mCtrlPoint[0] - mCtrlPoint[1]).norm();
    if(firstTwoDist < resolution * 0.0001){
        segments.push_back(this->mCtrlPoint);
        return segments;
    }

    Scalar curveLength = this->GetLength(0,1);

    // For singular cases
    if(curveLength < resolution){
        segments.push_back(this->mCtrlPoint);
        return segments;
    }

    // Get a curve polyline for given resolution
    int np = 1 + (curveLength / resolution);

    std::vector<Vector3> pts;
    this->SubdivideByLength(np, pts);

    for(int i = 0; i < np - 1; i++)
    {
        std::vector<Vector3> segment;

        segment.push_back(pts[i]);
        segment.push_back(pts[i+1]);

        segments.push_back(segment);
    }

    return segments;
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
//template
//class NURBSCurve<float>;

template
class NURBSCurve<double>;
//----------------------------------------------------------------------------
}
