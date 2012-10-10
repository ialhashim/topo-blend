#include "NURBSCurve.h"

template <typename Real>
NURBSCurve<Real>::NURBSCurve (std::vector<Vector3> ctrlPoint, std::vector<Real> ctrlWeight, int degree, bool loop, bool open) : SingleCurve<Real>((Real)0, (Real)1), mLoop(loop)
{
	int numCtrlPoints = ctrlPoint.size();

	assert(numCtrlPoints >= 2);
	assert(1 <= degree && degree <= numCtrlPoints-1);

	mNumCtrlPoints = numCtrlPoints;
	mReplicate = (loop ? (open ? 1 : degree) : 0);
	CreateControl(ctrlPoint, ctrlWeight);
	mBasis.Create(mNumCtrlPoints + mReplicate, degree, open);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSCurve<Real>::~NURBSCurve ()
{
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::CreateControl (std::vector<Vector3> ctrlPoint, std::vector<Real> ctrlWeight)
{
	int newNumCtrlPoints = mNumCtrlPoints + mReplicate;
    newNumCtrlPoints = newNumCtrlPoints;

	mCtrlPoint = ctrlPoint;
	mCtrlWeight = ctrlWeight;
}

//----------------------------------------------------------------------------
template <typename Real>
int NURBSCurve<Real>::GetNumCtrlPoints ()
{
    return mNumCtrlPoints;
}
//----------------------------------------------------------------------------
template <typename Real>
int NURBSCurve<Real>::GetDegree ()
{
    return mBasis.GetDegree();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsOpen ()
{
    return mBasis.IsOpen();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsUniform ()
{
    return mBasis.IsUniform();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSCurve<Real>::IsLoop ()
{
    return mLoop;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::SetControlPoint (int i,  Vector3& ctrl)
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
Vector3 NURBSCurve<Real>::GetControlPoint (int i)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlPoint[i];
    }

    return Vector3(MAX_REAL, MAX_REAL, MAX_REAL);
}

template <typename Real>
std::vector<Vector3> NURBSCurve<Real>::getControlPoints()
{
	return mCtrlPoint;
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
Real NURBSCurve<Real>::GetControlWeight (int i)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlWeight[i];
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSCurve<Real>::SetKnot (int i, Real knot)
{
    mBasis.SetKnot(i, knot);
}
//----------------------------------------------------------------------------
template <typename Real>
Real NURBSCurve<Real>::GetKnot (int i)
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
    Vector3 X = Vector3(0);
    Real w = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD0(i)*mCtrlWeight[i];
        X += mCtrlPoint[i]*tmp;
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
    Vector3 XDer1 = Vector3(0);
    Real wDer1 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD1(i)*mCtrlWeight[i];
        XDer1 += mCtrlPoint[i]*tmp;
        wDer1 += tmp;
    }
    Vector3 PDer1 = invW*(XDer1 - P*wDer1);
    if (der1)
    {
        *der1 = PDer1;
    }

    if (!der2 && !der3)
    {
        return;
    }

    // Compute second derivative.
    Vector3 XDer2 = Vector3(0);
    Real wDer2 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD2(i)*mCtrlWeight[i];
        XDer2 += mCtrlPoint[i]*tmp;
        wDer2 += tmp;
    }
    Vector3 PDer2 = invW*(XDer2 - PDer1*((Real)2)*wDer1 - P*wDer2);
    if (der2)
    {
        *der2 = PDer2;
    }

    if (!der3)
    {
        return;
    }

    // Compute third derivative.
    Vector3 XDer3 = Vector3(0);
    Real wDer3 = (Real)0;
    for (i = imin; i <= imax; ++i)
    {
        tmp = mBasis.GetD3(i)*mCtrlWeight[i];
        XDer3 += mCtrlPoint[i]*tmp;
        wDer3 += tmp;
    }
    if (der3)
    {
        *der3 = invW*(XDer3 - PDer2*((Real)3)*wDer1 -
            PDer1*((Real)3)*wDer2 - P*wDer3);
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

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template 
class NURBSCurve<float>;

template
class NURBSCurve<double>;
//----------------------------------------------------------------------------
