#include "NURBSRectangle.h"

//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::NURBSRectangle (int numUCtrlPoints,
    int numVCtrlPoints, Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
    :
    ParametricSurface<Real>((Real)0, (Real)1, (Real)0, (Real)1, true)
{
    assert(numUCtrlPoints >= 2);
    assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1);
    assert(numVCtrlPoints >= 2);
    assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1);

    mLoop[0] = uLoop;
    mLoop[1] = vLoop;

    mNumUCtrlPoints = numUCtrlPoints;
    mNumVCtrlPoints = numVCtrlPoints;
    mUReplicate = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mVReplicate = (vLoop ? (vOpen ? 1 : vDegree) : 0);
    CreateControl(ctrlPoint, ctrlWeight);

    mBasis[0].Create(mNumUCtrlPoints + mUReplicate, uDegree, uOpen);
    mBasis[1].Create(mNumVCtrlPoints + mVReplicate, vDegree, vOpen);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::NURBSRectangle (int numUCtrlPoints,
    int numVCtrlPoints, Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, bool uOpen, Real* vKnot)
    :
    ParametricSurface<Real>((Real)0, (Real)1, (Real)0, (Real)1, true)
{
    assert(numUCtrlPoints >= 2);
    assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assert(numVCtrlPoints >= 2);
    assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
        "Invalid input\n");

    mLoop[0] = uLoop;
    mLoop[1] = vLoop;

    mNumUCtrlPoints = numUCtrlPoints;
    mNumVCtrlPoints = numVCtrlPoints;
    mUReplicate = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mVReplicate = (vLoop ? 1 : 0);
    CreateControl(ctrlPoint, ctrlWeight);

    mBasis[0].Create(mNumUCtrlPoints + mUReplicate, uDegree, uOpen);
    mBasis[1].Create(mNumVCtrlPoints + mVReplicate, vDegree, vKnot);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::NURBSRectangle (int numUCtrlPoints,
    int numVCtrlPoints, Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, Real* uKnot, bool vOpen)
    :
    ParametricSurface<Real>((Real)0, (Real)1, (Real)0, (Real)1, true)
{
    assert(numUCtrlPoints >= 2);
    assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assert(numVCtrlPoints >= 2);
    assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
        "Invalid input\n");

    mLoop[0] = uLoop;
    mLoop[1] = vLoop;

    mNumUCtrlPoints = numUCtrlPoints;
    mNumVCtrlPoints = numVCtrlPoints;
    mUReplicate = (uLoop ? 1 : 0);
    mVReplicate = (vLoop ? (vOpen ? 1 : vDegree) : 0);
    CreateControl(ctrlPoint, ctrlWeight);

    mBasis[0].Create(mNumUCtrlPoints + mUReplicate, uDegree, uKnot);
    mBasis[1].Create(mNumVCtrlPoints + mVReplicate, vDegree, vOpen);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::NURBSRectangle (int numUCtrlPoints,
    int numVCtrlPoints, Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, Real* uKnot,
    Real* vKnot)
    :
    ParametricSurface<Real>((Real)0, (Real)1, (Real)0, (Real)1, true)
{
    assert(numUCtrlPoints >= 2);
    assert(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assert(numVCtrlPoints >= 2);
    assert(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
        "Invalid input\n");

    mLoop[0] = uLoop;
    mLoop[1] = vLoop;

    mNumUCtrlPoints = numUCtrlPoints;
    mNumVCtrlPoints = numVCtrlPoints;
    mUReplicate = (uLoop ? 1 : 0);
    mVReplicate = (vLoop ? 1 : 0);
    CreateControl(ctrlPoint,ctrlWeight);

    mBasis[0].Create(mNumUCtrlPoints + mUReplicate, uDegree, uKnot);
    mBasis[1].Create(mNumVCtrlPoints + mVReplicate, vDegree, vKnot);
}
//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::~NURBSRectangle ()
{
    //delete2(mCtrlPoint);
    //delete2(mCtrlWeight);
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::CreateControl (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight)
{
    int newNumUCtrlPoints = mNumUCtrlPoints + mUReplicate;
    int newNumVCtrlPoints = mNumVCtrlPoints + mVReplicate;

    mCtrlPoint = ctrlPoint;
    mCtrlWeight = ctrlWeight;
}
//----------------------------------------------------------------------------
template <typename Real>
int NURBSRectangle<Real>::GetNumCtrlPoints (int dim) const
{
    return mBasis[dim].GetNumCtrlPoints();
}
//----------------------------------------------------------------------------
template <typename Real>
int NURBSRectangle<Real>::GetDegree (int dim) const
{
    return mBasis[dim].GetDegree();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSRectangle<Real>::IsOpen (int dim) const
{
    return mBasis[dim].IsOpen();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSRectangle<Real>::IsUniform (int dim) const
{
    return mBasis[dim].IsUniform();
}
//----------------------------------------------------------------------------
template <typename Real>
bool NURBSRectangle<Real>::IsLoop (int dim) const
{
    return mLoop[dim];
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::SetControlPoint (int uIndex, int vIndex,
    const Vector3& ctrl)
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        // Set the control point.
        mCtrlPoint[uIndex][vIndex] = ctrl;

        // Set the replicated control point.
        bool doUReplicate = (uIndex < mUReplicate);
        bool doVReplicate = (vIndex < mVReplicate);
        int uExt = 0, vExt = 0;

        if (doUReplicate)
        {
            uExt = mNumUCtrlPoints + uIndex;
            mCtrlPoint[uExt][vIndex] = ctrl;
        }
        if (doVReplicate)
        {
            vExt = mNumVCtrlPoints + vIndex;
            mCtrlPoint[uIndex][vExt] = ctrl;
        }
        if (doUReplicate && doVReplicate)
        {
            mCtrlPoint[uExt][vExt] = ctrl;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::GetControlPoint (int uIndex,
    int vIndex) const
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        return mCtrlPoint[uIndex][vIndex];
    }

    return Vector3(MAX_REAL, MAX_REAL, MAX_REAL);
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::SetControlWeight (int uIndex, int vIndex,
    Real weight)
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        // Set the control weight.
        mCtrlWeight[uIndex][vIndex] = weight;

        // Set the replicated control weight.
        bool doUReplicate = (uIndex < mUReplicate );
        bool doVReplicate = (vIndex < mVReplicate);
        int uExt = 0, vExt = 0;

        if (doUReplicate)
        {
            uExt = mNumUCtrlPoints + uIndex;
            mCtrlWeight[uExt][vIndex] = weight;
        }
        if (doVReplicate)
        {
            vExt = mNumVCtrlPoints + vIndex;
            mCtrlWeight[uIndex][vExt] = weight;
        }
        if (doUReplicate && doVReplicate)
        {
            mCtrlWeight[uExt][vExt] = weight;
        }
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Real NURBSRectangle<Real>::GetControlWeight (int uIndex, int vIndex) const
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        return mCtrlWeight[uIndex][vIndex];
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::SetKnot (int dim, int i, Real knot)
{
    if (0 <= dim && dim <= 1)
    {
        mBasis[dim].SetKnot(i,knot);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Real NURBSRectangle<Real>::GetKnot (int dim, int i) const
{
    if (0 <= dim && dim <= 1)
    {
        return mBasis[dim].GetKnot(i);
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::Get (Real u, Real v, Vector3& pos,
    Vector3& derU, Vector3& derV, Vector3& derUU,
    Vector3& derUV, Vector3& derVV) const
{
    int iu, iumin, iumax;
    if (derUU)
    {
        mBasis[0].Compute(u, 0, iumin, iumax);
        mBasis[0].Compute(u, 1, iumin, iumax);
        mBasis[0].Compute(u, 2, iumin, iumax);
    }
    else if (derUV || derU)
    {
        mBasis[0].Compute(u, 0, iumin, iumax);
        mBasis[0].Compute(u, 1, iumin, iumax);
    }
    else
    {
        mBasis[0].Compute(u, 0, iumin, iumax);
    }

    int iv, ivmin, ivmax;
    if (derVV)
    {
        mBasis[1].Compute(v, 0, ivmin, ivmax);
        mBasis[1].Compute(v, 1, ivmin, ivmax);
        mBasis[1].Compute(v, 2, ivmin, ivmax);
    }
    else if (derUV || derV)
    {
        mBasis[1].Compute(v, 0, ivmin, ivmax);
        mBasis[1].Compute(v, 1, ivmin, ivmax);
    }
    else
    {
        mBasis[1].Compute(v, 0, ivmin, ivmax);
    }

    Real tmp;

    Vector3 X = Vector3(0);
    Real w = (Real)0;
    for (iu = iumin; iu <= iumax; ++iu)
    {
        for (iv = ivmin; iv <= ivmax; ++iv)
        {
            tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD0(iv)*mCtrlWeight[iu][iv];
            X += mCtrlPoint[iu][iv]*tmp;
            w += tmp;
        }
    }
    Real invW = ((Real)1)/w;
    Vector3 P = invW*X;
    if (pos)
    {
        pos = P;
    }

    if (!derU && !derV && !derUU && !derUV && !derVV)
    {
        return;
    }

    Real wDerU = (Real)0;
    Real wDerV = (Real)0;
    Vector3 PDerU = Vector3(0);
    Vector3 PDerV = Vector3(0);

    if (derU || derUU || derUV)
    {
        Vector3 XDerU = Vector3(0);
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD0(iv)*
                    mCtrlWeight[iu][iv];
                XDerU += mCtrlPoint[iu][iv]*tmp;
                wDerU += tmp;
            }
        }
        PDerU = invW*(XDerU - P*wDerU);
        if (derU)
        {
            derU = PDerU;
        }
    }

    if (derV || derVV || derUV)
    {
        Vector3 XDerV = Vector3(0);
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD1(iv) * mCtrlWeight[iu][iv];
                XDerV += mCtrlPoint[iu][iv]*tmp;
                wDerV += tmp;
            }
        }
        PDerV = invW*(XDerV - P*wDerV);
        if (derV)
        {
            derV = PDerV;
        }
    }

    if (!derUU && !derUV && !derVV)
    {
        return;
    }

    if (derUU)
    {
        Vector3 XDerUU = Vector3(0);
        Real wDerUU = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD2(iu)*mBasis[1].GetD0(iv)*
                    mCtrlWeight[iu][iv];
                XDerUU += mCtrlPoint[iu][iv]*tmp;
                wDerUU += tmp;
            }
        }
        derUU = invW*(XDerUU - (PDerU*(Real)2)*wDerU - P*wDerUU);
    }

    if (derUV)
    {
        Vector3 XDerUV = Vector3(0);
        Real wDerUV = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD1(iv)*
                    mCtrlWeight[iu][iv];
                XDerUV += mCtrlPoint[iu][iv]*tmp;
                wDerUV += tmp;
            }
        }
        derUV = invW*(XDerUV - PDerV*wDerU - PDerU*wDerV - P*wDerUV);
    }

    if (derVV)
    {
        Vector3 XDerVV = Vector3(0);
        Real wDerVV = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD2(iv) * mCtrlWeight[iu][iv];
                XDerVV += mCtrlPoint[iu][iv]*tmp;
                wDerVV += tmp;
            }
        }
        derVV = invW*(XDerVV - PDerV*((Real)2)*wDerV - P*wDerVV);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::P (Real u, Real v) const
{
    Vector3 pos;
    Get(u, v, pos);
    return pos;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PU (Real u, Real v) const
{
    Vector3 derU;
    Get(u, v, Vector3(0), derU);
    return derU;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PV (Real u, Real v) const
{
    Vector3 derV;
    Get(u, v, Vector3(0), Vector3(0), derV);
    return derV;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PUU (Real u, Real v) const
{
    Vector3 derUU;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), derUU);
    return derUU;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PUV (Real u, Real v) const
{
    Vector3 derUV;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), Vector3(0), derUV);
    return derUV;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PVV (Real u, Real v) const
{
    Vector3 derVV;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), Vector3(0), Vector3(0), derVV);
    return derVV;
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::generateSurfaceQuads( int resolution )
{
	double res = resolution;
	double du = 1.0 / res;
	double dv = 1.0 / res;

	for(int i = 0; i < res; i++){
		double u = double(i) / res;

		for(int j = 0; j < res; j++){
			double v = double(j) / res;

			std::vector<Vec3d> pos = std::vector<Vec3d>(4, Vec3d(0));
			std::vector<Vec3d> dU = pos, dV = pos;

			Get(u,v,		pos[0], dU[0], dV[0]);
			Get(u+du,v,		pos[1], dU[1], dV[1]);
			Get(u+du,v+dv,	pos[2], dU[2], dV[2]);
			Get(u,v+dv,		pos[3], dU[3], dV[3]);

			SurfaceQuad quad;

			for(int k = 0; k < 4; k++){
				quad.p[k] = pos[k];
				quad.n[k] = cross(dU[k], dV[k]).normalized();
			}

			quads.push_back(quad);
		}
	}
}

//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
template
class NURBSRectangle<float>;

template
class NURBSRectangle<double>;
//----------------------------------------------------------------------------

