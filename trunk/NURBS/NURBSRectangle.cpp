// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)


#include "NURBSRectangle.h"
#include "NURBSCurve.h"

namespace NURBS
{

//----------------------------------------------------------------------------
template <typename Real>
NURBSRectangle<Real>::NURBSRectangle (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen)
    :
    ParametricSurface<Real>((Real)0, (Real)1, (Real)0, (Real)1, true)
{
    int numUCtrlPoints = ctrlPoint.size();
    int numVCtrlPoints = ctrlPoint.front().size();

    assertion(numUCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assertion(numVCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
        "Invalid input\n");

	mLoop.resize(2);
    mLoop[0] = uLoop;
    mLoop[1] = vLoop;

    mNumUCtrlPoints = numUCtrlPoints;
    mNumVCtrlPoints = numVCtrlPoints;
    mUReplicate = (uLoop ? (uOpen ? 1 : uDegree) : 0);
    mVReplicate = (vLoop ? (vOpen ? 1 : vDegree) : 0);
    CreateControl(ctrlPoint, ctrlWeight);

	mBasis.resize(2);

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
    assertion(numUCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assertion(numVCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
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
    assertion(numUCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assertion(numVCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
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
    assertion(numUCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= uDegree && uDegree <= numUCtrlPoints - 1,
        "Invalid input\n");
    assertion(numVCtrlPoints >= 2, "Invalid input\n");
    assertion(1 <= vDegree && vDegree <= numVCtrlPoints - 1,
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
NURBSRectangle<Real> NURBSRectangle<Real>::createSheet(Scalar width, Scalar length, Vector3 center, Vector3 dU, Vector3 dV, int nU, int nV)
{
    int degree = 3;

    Vector3 corner = center - (dU * width * 0.5) - (dV * length * 0.5);

    Scalar aspect_U = 1.0, aspect_V = 1.0;
    if(width > length) aspect_U = length / width;
    if(length > width) aspect_V = width / length;

    nU *= 1.0 / aspect_U;
    nV *= 1.0 / aspect_V;

    // Rectangular surface
    std::vector< Array1D_Vector3 > pts( nU, Array1D_Vector3( nV, Vector3(0,0,0) ) );
    std::vector< std::vector<Scalar> > weights( nU, std::vector<Scalar>( nV, 1.0 ) );

    Vector3 deltaU = (width  / (nU-1)) * dU;
    Vector3 deltaV = (length / (nV-1)) * dV;

    for(int y = 0; y < nV; y++){
        for(int x = 0; x < nU; x++)
        {
            pts[x][y] = corner + (deltaU * x) + (deltaV * y);
        }
    }

    return NURBSRectangle<Real>(pts, weights, degree, degree, false, false, true, true);
}

template <typename Real>
NURBSRectangle<Real> NURBSRectangle<Real>::createSheet( Vector3d corner1, Vector3d corner2, int stepsU, int stepsV )
{
    int nU = stepsU, nV = stepsV;
    int degree = 3;

    Vector3 center = (corner1 + corner2) * 0.5;
    Vector3 corner = corner1;

    Vector3 d = corner1 - corner2;
    assert(d.norm() > 0);

    QVector<Scalar> vals;
    vals.push_back(fabs(d.x()));
    vals.push_back(fabs(d.y()));
    vals.push_back(fabs(d.z()));
    qSort(vals);
    if(vals[1] == 0.0) vals[1] = 1e-16;
    if(vals[1] == vals[2]) vals[2] += 1e-6;

    double width = vals[2];
    double length = vals[1];

    Vector3 xyz[3] = { Vector3(0,0,1), Vector3(0,1,0), Vector3(1,0,0) };
    Vector3 dU = xyz[ vals.indexOf(width ) ];
    Vector3 dV = xyz[ vals.indexOf(length) ];

    Scalar aspect_U = 1.0, aspect_V = 1.0;
    if(width > length) aspect_U = length / width;
    if(length > width) aspect_V = width / length;

    nU *= 1.0 / aspect_U;
    nV *= 1.0 / aspect_V;

    // Rectangular surface
    std::vector< std::vector<Vector3> > pts( nU, std::vector<Vector3>( nV, Vector3(0,0,0) ) );
    std::vector< std::vector<Scalar> > weights( nU, std::vector<Scalar>( nV, 1.0 ) );

    Vector3 deltaU = (width  / (nU-1)) * dU;
    Vector3 deltaV = (length / (nV-1)) * dV;

    for(int y = 0; y < nV; y++){
        for(int x = 0; x < nU; x++)
        {
            pts[x][y] = corner + (deltaU * x) + (deltaV * y);
        }
    }

    return NURBSRectangle<Real>(pts, weights, degree, degree, false, false, true, true);
}

template <typename Real>
NURBSRectangle<Real> NURBS::NURBSRectangle<Real>::createSheetFromPoints( Array2D_Vector3 ctrlPoint )
{
	int degree = 3;
	int nU = ctrlPoint.size(), nV = ctrlPoint.front().size();
	std::vector< std::vector<Scalar> > weights( nU, std::vector<Scalar>( nV, 1.0 ) );
	return NURBSRectangle<Real>(ctrlPoint, weights, degree, degree, false, false, true, true); 
}

//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::CreateControl (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight)
{
    int newNumUCtrlPoints = mNumUCtrlPoints + mUReplicate;
    int newNumVCtrlPoints = mNumVCtrlPoints + mVReplicate;

    mCtrlPoint = new2<Vector3>(newNumUCtrlPoints, newNumVCtrlPoints, Vector3(0,0,0));
    mCtrlWeight = new2<Real>(newNumUCtrlPoints, newNumVCtrlPoints, 1.0);

    for (int iu = 0; iu < newNumUCtrlPoints; iu++)
    {
        int uOld = iu % mNumUCtrlPoints;
        for (int iv = 0; iv < newNumVCtrlPoints; iv++)
        {
            int vOld = iv % mNumVCtrlPoints;
            mCtrlPoint[iu][iv] = ctrlPoint[uOld][vOld];
            mCtrlWeight[iu][iv] = ctrlWeight[uOld][vOld];
        }
    }
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

    return Vector3(std::numeric_limits<Real>::max(), std::numeric_limits<Real>::max(),
        std::numeric_limits<Real>::max());
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

    return std::numeric_limits<Real>::max();
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

    return std::numeric_limits<Real>::max();
}
//----------------------------------------------------------------------------
template <typename Real>
void NURBSRectangle<Real>::Get (Real u, Real v, Vector3* pos,
    Vector3* derU, Vector3* derV, Vector3* derUU,
    Vector3* derUV, Vector3* derVV)
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

    Vector3 X = Vector3_ZERO;
    Real w = (Real)0;
    for (iu = iumin; iu <= iumax; ++iu)
    {
        for (iv = ivmin; iv <= ivmax; ++iv)
        {
            tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD0(iv)*mCtrlWeight[iu][iv];
            X += tmp*mCtrlPoint[iu][iv];
            w += tmp;
        }
    }
    Real invW = ((Real)1)/w;
    Vector3 P = invW*X;
    if (pos)
    {
        *pos = P;
    }

    if (!derU && !derV && !derUU && !derUV && !derVV)
    {
        return;
    }

    Real wDerU = (Real)0;
    Real wDerV = (Real)0;
    Vector3 PDerU = Vector3_ZERO;
    Vector3 PDerV = Vector3_ZERO;

    if (derU || derUU || derUV)
    {
        Vector3 XDerU = Vector3_ZERO;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD0(iv)*
                    mCtrlWeight[iu][iv];
                XDerU += tmp*mCtrlPoint[iu][iv];
                wDerU += tmp;
            }
        }
        PDerU = invW*(XDerU - wDerU*P);
        if (derU)
        {
            *derU = PDerU;
        }
    }

    if (derV || derVV || derUV)
    {
        Vector3 XDerV = Vector3_ZERO;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD1(iv)*
                    mCtrlWeight[iu][iv];
                XDerV += tmp*mCtrlPoint[iu][iv];
                wDerV += tmp;
            }
        }
        PDerV = invW*(XDerV - wDerV*P);
        if (derV)
        {
            *derV = PDerV;
        }
    }

    if (!derUU && !derUV && !derVV)
    {
        return;
    }

    if (derUU)
    {
        Vector3 XDerUU = Vector3_ZERO;
        Real wDerUU = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD2(iu)*mBasis[1].GetD0(iv)*
                    mCtrlWeight[iu][iv];
                XDerUU += tmp*mCtrlPoint[iu][iv];
                wDerUU += tmp;
            }
        }
        *derUU = invW*(XDerUU - ((Real)2)*wDerU*PDerU - wDerUU*P);
    }

    if (derUV)
    {
        Vector3 XDerUV = Vector3_ZERO;
        Real wDerUV = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD1(iv)*
                    mCtrlWeight[iu][iv];
                XDerUV += tmp*mCtrlPoint[iu][iv];
                wDerUV += tmp;
            }
        }
        *derUV = invW*(XDerUV - wDerU*PDerV - wDerV*PDerU - wDerUV*P);
    }

    if (derVV)
    {
        Vector3 XDerVV = Vector3_ZERO;
        Real wDerVV = (Real)0;
        for (iu = iumin; iu <= iumax; ++iu)
        {
            for (iv = ivmin; iv <= ivmax; ++iv)
            {
                tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD2(iv)*
                    mCtrlWeight[iu][iv];
                XDerVV += tmp*mCtrlPoint[iu][iv];
                wDerVV += tmp;
            }
        }
        *derVV = invW*(XDerVV - ((Real)2)*wDerV*PDerV - wDerVV*P);
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::P (Real u, Real v)
{
    Vector3 pos;
    Get(u, v, &pos, 0, 0, 0, 0, 0);
    return pos;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PU (Real u, Real v)
{
    Vector3 derU;
    Get(u, v, 0, &derU, 0, 0, 0, 0);
    return derU;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PV (Real u, Real v)
{
    Vector3 derV;
    Get(u, v, 0, 0, &derV, 0, 0, 0);
    return derV;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PUU (Real u, Real v)
{
    Vector3 derUU;
    Get(u, v, 0, 0, 0, &derUU, 0, 0);
    return derUU;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PUV (Real u, Real v)
{
    Vector3 derUV;
    Get(u, v, 0, 0, 0, 0, &derUV, 0);
    return derUV;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector3 NURBSRectangle<Real>::PVV (Real u, Real v)
{
    Vector3 derVV;
    Get(u, v, 0, 0, 0, 0, 0, &derVV);
    return derVV;
}
//----------------------------------------------------------------------------

template <typename Real>
std::vector< std::vector<Vector3> > NURBSRectangle<Real>::generateSurfaceTris( Scalar resolution )
{
    std::vector< std::vector<Vector3> > tris;

    std::vector<Real> valU,valV;
    uniformCoordinates(valU, valV, resolution);

    for(int y = 0; y < (int)valV.size() - 1; y++)
    {
        for(int x = 0; x < (int)valU.size() - 1; x++)
        {
            std::vector<Vector3> pos(4);

            pos[0] = P(valU[x]	, valV[y]	);
            pos[1] = P(valU[x+1], valV[y]	);
            pos[2] = P(valU[x+1], valV[y+1]	);
            pos[3] = P(valU[x]	, valV[y+1]	);

            std::vector<Vector3> f1,f2;

            f1.push_back(pos[0]); f1.push_back(pos[2]); f1.push_back(pos[1]);
            f2.push_back(pos[0]); f2.push_back(pos[3]); f2.push_back(pos[2]);

            tris.push_back(f1);
            tris.push_back(f2);
        }
    }

    return tris;
}

template <typename Real>
void NURBSRectangle<Real>::generateSurfaceQuads(double resolution)
{
    std::vector<Real> valU,valV;
    uniformCoordinates(valU, valV, resolution);

    for(int y = 0; y < (int)valV.size() - 1; y++)
    {
        for(int x = 0; x < (int)valU.size() - 1; x++)
        {
            std::vector<Vector3> pos = Array1D_Vector3(4, Vector3(0,0,0));
            std::vector<Vector3> dU = pos, dV = pos, normal = pos;

            this->GetFrame(valU[x]	, valV[y]	,	pos[0], dU[0], dV[0], normal[0]);
            this->GetFrame(valU[x+1]	, valV[y]	,	pos[1], dU[1], dV[1], normal[1]);
            this->GetFrame(valU[x+1]	, valV[y+1]	,	pos[2], dU[2], dV[2], normal[2]);
            this->GetFrame(valU[x]	, valV[y+1]	,	pos[3], dU[3], dV[3], normal[3]);

            SurfaceQuad quad;

            for(int k = 0; k < 4; k++){
                quad.p[k] = pos[k];
                quad.n[k] = normal[k];
            }

            quads.push_back(quad);
        }
    }
}

template <typename Real>
void NURBSRectangle<Real>::uniformCoordinates( std::vector<Real> & valU, std::vector<Real> & valV, double resolution, int u, int v )
{
    std::vector<Vector3> cptsU = GetControlPointsU(u);
    std::vector<Vector3> cptsV = GetControlPointsV(v);

    // Uniform across given U, V
    NURBSCurve<Real> curveU( cptsU, std::vector<Scalar>(cptsU.size(), 1) );
    NURBSCurve<Real> curveV( cptsV, std::vector<Scalar>(cptsV.size(), 1) );

    double lengthU = curveU.GetLength(0,1), lengthV = curveV.GetLength(0,1);

    // In case of zero sheet, which could be a curve
    if (lengthU < resolution || lengthV < resolution)
        return;

    double resU = resolution, resV = resolution;

    int nU = lengthU / resolution;
    int nV = lengthV / resolution;
    resU = lengthU / nU;
    resV = lengthV / nV;

    for(int i = 0; i <= nU; i++)
    {
        double distance = resU * i;
        valU.push_back(curveU.GetTime(distance));
    }
    for(int i = 0; i <= nV; i++)
    {
        double distance = resV * i;
        valV.push_back(curveV.GetTime(distance));
    }
}

template <typename Real>
void NURBSRectangle<Real>::generateSurfacePointsCoords( Scalar stepSize, std::vector< Array1D_Vector4d > & points )
{
    std::vector<Real> valU,valV;
    uniformCoordinates(valU, valV, stepSize);

    points.resize(valU.size(), Array1D_Vector4d(valV.size()));

    for(int y = 0; y < (int)valV.size(); y++)
        for(int x = 0; x < (int)valU.size(); x++)
            points[x][y] = Vector4d(valU[x], valV[y], 0, 0);
}

template <typename Real>
std::vector<Vector3> NURBSRectangle<Real>::GetControlPointsV( int vIndex )
{
    return mCtrlPoint[vIndex];
}

template <typename Real>
std::vector<Vector3> NURBSRectangle<Real>::GetControlPointsU( int uIndex )
{
    std::vector<Vector3> pts;

	for(int i = 0; i < (int)mCtrlPoint.size(); i++)
		pts.push_back(mCtrlPoint[i][uIndex]);

	return pts;
}


template <typename Real>
std::vector<Scalar> NURBS::NURBSRectangle<Real>::GetControlWeightsU( int uIndex )
{
	std::vector<Scalar> weights;

	for(int i = 0; i < (int)mCtrlWeight.size(); i++)
		weights.push_back(mCtrlWeight[i][uIndex]);


	return weights;
}

template <typename Real>
std::vector<Scalar> NURBS::NURBSRectangle<Real>::GetControlWeightsV( int vIndex )
{
	return mCtrlWeight[vIndex];
}


template <typename Real>
Array1D_Vector3 NURBSRectangle<Real>::intersect( NURBSRectangle<Real> & other, double resolution,
    Array1D_Vector4d & coordMe, Array1D_Vector4d & coordOther )
{
    Array1D_Vector3 samples;

    std::vector< std::vector<Vector3> > pnts1, pnts2;
    std::vector< std::vector< std::vector<Real> > > val( 2, std::vector< std::vector<Real> >(2) );
    std::vector<Real> valU1,valV1,valU2,valV2;
    std::vector<Real> iU1,iV1,iU2,iV2;

    generateSurfacePoints(resolution, pnts1, valU1, valV1);
    other.generateSurfacePoints(resolution, pnts2, valU2, valV2);

    val[0][0] = valU1; val[0][1] = valV1;
    val[1][0] = valU2; val[1][1] = valV2;

    std::vector< std::pair< std::pair<Scalar,Scalar>, Vector3d> > mysamples, othersamples;

    for(int y1 = 0; y1 < (int)valV1.size(); y1++){
        for(int x1 = 0; x1 < (int)valU1.size(); x1++){
            mysamples.push_back(std::make_pair( std::make_pair(valU1[x1], valV1[y1]), pnts1[x1][y1] ));
        }
    }

    for(int y2 = 0; y2 < (int)valV2.size(); y2++){
        for(int x2 = 0; x2 < (int)valU2.size(); x2++){
            othersamples.push_back(std::make_pair( std::make_pair(valU2[x2], valV2[y2]), pnts2[x2][y2] ));
        }
    }

    for(int i = 0; i < (int)mysamples.size(); i++)
    {
        for(int j = 0; j < (int)othersamples.size(); j++)
        {
            if(sphereTest(mysamples[i].second, othersamples[j].second, resolution * 0.5, resolution * 0.5))
                samples.push_back(mysamples[i].second);
        }
    }

    std::vector<size_t> corner_xrefs;
    weld(samples, corner_xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

    Vector3d p(0,0,0);
    double threshold = resolution * 0.5;

    // Project onto other surface
    Array1D_Vector4d otheruv = other.timeAt(samples, threshold);
    Array1D_Vector3 projectionOther;
    for(int i = 0; i < (int)otheruv.size(); i++){
        other.Get(otheruv[i][0], otheruv[i][1], &p);
        projectionOther.push_back(p);
    }

    samples.clear();

    // Project on me
    coordMe = this->timeAt(projectionOther, threshold);
    Array1D_Vector3 projectionMe;
    for(int i = 0; i < (int)coordMe.size(); i++){
        this->Get(coordMe[i][0], coordMe[i][1], &p);
        samples.push_back(p);
    }

    weld(samples, corner_xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

    // Cluster and average
    std::vector<bool> visited(samples.size(), false);
    QMap<int, QVector<Vector3d> > group;
    for(int i = 0; i < (int)samples.size(); i++){
        if(!visited[i]){
            for(int j = 0; j < (int)samples.size(); j++){
                if((samples[i] - samples[j]).norm() < resolution * 2){
                    samples[j] = samples[i];
                    visited[j] = true;
                    group[i].push_back(samples[j]);
                }
            }
            group[i].push_back(samples[i]);
            visited[i] = true;
        }
    }
    samples.clear();
    foreach(QVector<Vector3d> pnts, group.values()){
        Vector3d avg(0,0,0);
        foreach(Vector3d p, pnts){	avg += p; }
        avg /= pnts.size();
        samples.push_back(avg);
    }

    weld(samples, corner_xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

    Array1D_Vector3 clusterdSamples = samples;

    if(samples.size() == 0)
        return samples;

    coordMe = timeAt(samples, threshold);
    coordOther = other.timeAt(samples, threshold);
    return samples;
}

template <typename Real>
void NURBSRectangle<Real>::generateSurfacePoints( Scalar stepSize, std::vector< std::vector<Vector3> > & points,
                                                  std::vector<Real> & valU, std::vector<Real> & valV )
{
    uniformCoordinates(valU, valV, stepSize);

    points.resize(valU.size(), std::vector<Vector3>(valV.size()));

    for(int y = 0; y < (int)valV.size(); y++)
    {
        for(int x = 0; x < (int)valU.size(); x++)
        {
            points[x][y] = P(valU[x], valV[y]);
        }
    }
}

template <typename Real>
Vector4d NURBSRectangle<Real>::timeAt( const Vector3 & pos )
{
    std::vector< std::vector<Vector3> > pts;

    std::vector<Real> valU, valV;
    Scalar stepSize = 0.01 * (mCtrlPoint.front().front() - mCtrlPoint.back().back()).norm();
    generateSurfacePoints(stepSize, pts, valU, valV);

    int minIdxU = 0, minIdxV = 0;
    Scalar minDist = std::numeric_limits<Scalar>::max();

    // Approximate area search
    for(int x = 0; x < (int)valU.size(); x++){
        for(int y = 0; y < (int)valV.size(); y++){
            Scalar dist = (pts[x][y] - pos).norm();

            if(dist < minDist){
                minDist = dist;
                minIdxU = x;
                minIdxV = y;
            }
        }
    }

    // More precise search
    Vector4d minRange( valU[qMax(0, minIdxU - 1)], valV[qMax(0, minIdxV - 1)], 0, 0);
    Vector4d maxRange( valU[qMin((int)valU.size() - 1, minIdxU + 1)], valV[qMin((int)valV.size() - 1, minIdxV + 1)], 0, 0);

    Vector4d bestUV( valU[minIdxU], valV[minIdxV], 0, 0 );

    return timeAt(pos, bestUV, minRange, maxRange, minDist);
}

template <typename Real>
Vector4d NURBSRectangle<Real>::timeAt( const Vector3 & pos, Vector4d & bestUV, Vector4d & minRange, Vector4d & maxRange, Real currentDist, Real threshold )
{
    // 1) Subdivide
    int searchSteps = 10;
    Scalar du = (maxRange[0] - minRange[0]) / searchSteps;
    Scalar dv = (maxRange[1] - minRange[1]) / searchSteps;

    // 2) Compare all distances
    Scalar curU = bestUV[0], curV = bestUV[1];
    Scalar minDist = currentDist;

    for(int y = 0; y <= searchSteps; y++)
    {
        for(int x = 0; x <= searchSteps; x++)
        {
            Scalar u = minRange[0] + (x * du);
            Scalar v = minRange[1] + (y * dv);

            Vector3 curPos = P(u,v);

            Scalar dist = (curPos - pos).norm();

            if(dist < minDist){
                minDist = dist;
                curU = u;
                curV = v;
            }
        }
    }

    // 3) If minimum distance == current or less than threshold return best match
    if(minDist >= currentDist)
        return bestUV;
    else if(minDist < threshold)
        return Vector4d(curU, curV, 0, 0);
    else
    {
        // 4) Otherwise recursive search in smaller range
        Vector4d minRange( qMax(0.0, curU - du), qMax(0.0, curV - dv), 0, 0 );
        Vector4d maxRange( qMin(1.0, curU + du), qMin(1.0, curV + dv), 0, 0 );

        Vector4d crd(curU, curV, 0, 0);

        return timeAt(pos, crd, minRange, maxRange, minDist, threshold);
    }
}

template <typename Real>
Array1D_Vector4d NURBSRectangle<Real>::timeAt( const std::vector<Vector3> & positions, Real threshold )
{
    Array1D_Vector4d times;

    std::vector< std::vector<Vector3> > pts;
    std::vector<Real> valU, valV;
    Scalar stepSize = 0.01 * (mCtrlPoint.front().front() - mCtrlPoint.back().back()).norm();
    generateSurfacePoints(stepSize, pts, valU, valV);

    for(int i = 0; i < (int)positions.size(); i++)
    {
        Vector3 pos = positions[i];

        int minIdxU = 0, minIdxV = 0;
        Scalar minDist = std::numeric_limits<Scalar>::max();

        // Approximate area search
        for(int x = 0; x < (int)valU.size(); x++){
            for(int y = 0; y < (int)valV.size(); y++){
                Scalar dist = (pts[x][y] - pos).norm();

                if(dist < minDist){
                    minDist = dist;
                    minIdxU = x;
                    minIdxV = y;
                }
            }
        }

        // More precise search
        Vector4d minRange( valU[qMax(0, minIdxU - 1)], valV[qMax(0, minIdxV - 1)], 0, 0);
        Vector4d maxRange( valU[qMin((int)valU.size() - 1, minIdxU + 1)], valV[qMin((int)valV.size() - 1, minIdxV + 1)], 0, 0);

        Vector4d bestUV( valU[minIdxU], valV[minIdxV], 0, 0 );

        times.push_back( timeAt(pos, bestUV, minRange, maxRange, minDist, threshold) );
    }

    return times;
}

template <typename Real>
Vector4d NURBSRectangle<Real>::fastTimeAt( const Vector3 & pos )
{
	std::vector< std::vector<Vector3> > pts;
	std::vector<Real> valU, valV;
	Scalar stepSize = 0.1 * (mCtrlPoint.front().front() - mCtrlPoint.back().back()).norm();
	generateSurfacePoints(stepSize, pts, valU, valV);
	int minIdxU = 0, minIdxV = 0;
	Scalar minDist = std::numeric_limits<Scalar>::max();

	// Approximate area search
	for(int x = 0; x < (int)valU.size(); x++){
		for(int y = 0; y < (int)valV.size(); y++){
			Scalar dist = (pts[x][y] - pos).norm();
			if(dist < minDist){
				minDist = dist;
				minIdxU = x;
				minIdxV = y;
			}
		}
	}

	if(!valU.size() || !valV.size()) return Vector4d(0,0,0,0);

	Vector4d bestUV( valU[minIdxU], valV[minIdxV], 0, 0 );
	return bestUV;
}

template <typename Real>
std::vector< std::vector<Vector3> > NURBSRectangle<Real>::triangulateControlCage()
{
    int width = GetNumCtrlPoints(0);
    int length = GetNumCtrlPoints(1);

    std::vector< std::vector<Vector3> > tris;
    std::vector<Vector3> emptyTri(3, Vector3(0,0,0));

    // Draw surface of control cage
    for(int j = 0; j < length - 1; j++)
    {
        for(int i = 0; i < width - 1; i++)
        {
            // Triangle 1
            tris.push_back(emptyTri);
            tris.back()[0] = GetControlPoint(i,j);
            tris.back()[1] = GetControlPoint(i+1,j);
            tris.back()[2] = GetControlPoint(i,j+1);

            // Triangle 2
            tris.push_back(emptyTri);
            tris.back()[0] = GetControlPoint(i+1,j);
            tris.back()[1] = GetControlPoint(i+1,j+1);
            tris.back()[2] = GetControlPoint(i,j+1);
        }
    }

    return tris;
}

template <typename Real>
Vector3 NURBSRectangle<Real>::projectOnControl( Real u, Real v )
{
    std::vector< std::vector<Vector3> > tris = NURBSRectangle::triangulateControlCage();

    Vector3 pos(0,0,0), t0(0,0,0), t1(0,0,0), normal(0,0,0);
    this->GetFrame(u,v, pos, t0,t1, normal);
    normal.normalize();

    QMap<double, Vector3d> hits;
    Vector3d isect(0,0,0);

    foreach( std::vector<Vector3> tri, tris )
    {
        if(intersectRayTri(tri, pos, normal, isect))
            hits[ (isect - pos).norm() ] = isect;
    }

    if(hits.size())
        return hits.values().front();
    else{
        qDebug() << "No intersection at u = " << u << ", v = " << v;
        return pos;
    }
}

template <typename Real>
void NURBSRectangle<Real>::translate( const Vector3d & delta )
{
    for(int y = 0; y < (int)mCtrlPoint.size(); y++)
        for(int x = 0; x < (int)mCtrlPoint[0].size(); x++)
            mCtrlPoint[y][x] += delta;
}

template <typename Real>
void NURBSRectangle<Real>::scale( Scalar scaleFactor )
{
    for(int y = 0; y < (int)mCtrlPoint.size(); y++)
        for(int x = 0; x < (int)mCtrlPoint[0].size(); x++)
            mCtrlPoint[y][x] *= scaleFactor;
}

template <typename Real>
Array2D_Vector3 NURBSRectangle<Real>::swapUV( const Array2D_Vector3 & controlPoints )
{
	Array2D_Vector3 tmp(controlPoints.front().size(), Array1D_Vector3(controlPoints.size(), Vector3(0,0,0)));

	for(int i = 0; i < (int)tmp.size(); i++)
		for(int j = 0; j < (int)tmp.front().size(); j++)
			tmp[i][j] = controlPoints[j][i];
	
	return tmp;
}

template <typename Real>
void NURBSRectangle<Real>::refineU( Array1D_Real & insknts, Array2D_Vector3 & Qw )
{
	Qw.clear();
	Qw.resize( mNumUCtrlPoints + insknts.size(), Array1D_Vector3(mNumVCtrlPoints, Vector3(0,0,0)) );

	for(int v = 0; v < mNumVCtrlPoints; v++)
	{
		NURBSCurve<Real> curveU = NURBSCurve<Real>::createCurveFromPoints( GetControlPointsU(v) );
		
		Array1D_Vector3 _Qw;
		Array1D_Real _Ubar;
		curveU.refine(insknts, _Qw, _Ubar);

		for(int i = 0; i < (int)_Qw.size(); i++)
			Qw[i][v] = _Qw[i];
	}
}

template <typename Real>
void NURBSRectangle<Real>::refine( Array1D_Real & insknts, Array2D_Vector3 & Qw, int dir )
{
	NURBSRectangle<Real> rect = *this;

	// Swap on V
	if(dir != 0) rect = NURBSRectangle<Real>::createSheetFromPoints( swapUV(rect.mCtrlPoint) );
	
	rect.refineU(insknts, Qw);
}

template <typename Real>
Array2D_Vector3 NURBSRectangle<Real>::midPointRefined()
{
	Array2D_Vector3 Qw = mCtrlPoint;

	std::vector<Real> inskntsU;
	Array1D_Real oldKnotVecU = GetKnotVectorU(true);
	for(int i = 0; i < (int)oldKnotVecU.size() - 1; i++){
		double range = (oldKnotVecU[i+1] - oldKnotVecU[i]);
		inskntsU.push_back( (0.5 * range) + oldKnotVecU[i] );
	}
	refine(inskntsU, Qw, 0);

	NURBSRectangle<Real> rectRefinedU = NURBSRectangle<Real>::createSheetFromPoints( Qw );

	std::vector<Real> inskntsV;
	Array1D_Real oldKnotVecV = rectRefinedU.GetKnotVectorV(true);
	for(int i = 0; i < (int)oldKnotVecV.size() - 1; i++){
		double range = (oldKnotVecV[i+1] - oldKnotVecV[i]);
		inskntsV.push_back( (0.5 * range) + oldKnotVecV[i] );
	}
	rectRefinedU.refine(inskntsV, Qw, 1);

	return swapUV(Qw);
}

template <typename Real>
Array1D_Real NURBSRectangle<Real>::GetKnotVectorU(bool isInnerOnly)
{
	BSplineBasis<Real> b = mBasis[0];
	if(isInnerOnly){
		int d = this->GetDegree(0);
		Array1D_Real result(b.mKnot.begin() + d, b.mKnot.end() - d);
		return result;
	}
	else
		return b.mKnot;
}

template <typename Real>
Array1D_Real NURBSRectangle<Real>::GetKnotVectorV(bool isInnerOnly)
{
	BSplineBasis<Real> b = mBasis[1];
	if(isInnerOnly){
		int d = this->GetDegree(1);
		Array1D_Real result(b.mKnot.begin() + d, b.mKnot.end() - d);
		return result;
	}
	else
		return b.mKnot;
}

template <typename Real>
Array2D_Vector3 NURBSRectangle<Real>::simpleRefine( int k, int dir )
{
	if(k <= 0) return this->mCtrlPoint;

	NURBSRectangle<Real> curRect = *this;

	for(int i = 0; i < k; i++)
	{
		Array1D_Real knotVector = curRect.GetKnotVectorU(true);
		if(dir == 1) knotVector = curRect.GetKnotVectorV(true);

		Array1D_Vector3 midSegments = curRect.GetControlPointsU( 0.5 * curRect.mNumVCtrlPoints );
		if(dir == 1) midSegments = curRect.GetControlPointsV( 0.5 * curRect.mNumUCtrlPoints );

		// Find longest segment
		int segment = 0;
		double maxDist = -DBL_MAX;
		for(int j = 0; j < (int)midSegments.size() - 1; j++)
		{
			double dist = (midSegments[j+1] - midSegments[j]).norm();
			if(dist > maxDist){
				maxDist = dist;
				segment = j;
			}
		}

		double t = double(segment) / (midSegments.size()-2);

		int idx = t * (knotVector.size()-2);
		double range = knotVector[idx+1] - knotVector[idx];
		double u = (0.5 * range) + knotVector[idx];

		Array2D_Vector3 newPts;

        Array1D_Real insk = Array1D_Real(1,u);
        curRect.refine(insk, newPts, dir);
		if(dir == 1) newPts = NURBSRectangle<Real>::swapUV( newPts );

		curRect = NURBSRectangle<Real>::createSheetFromPoints( newPts );
	}

	return curRect.mCtrlPoint;
}

template <typename Real>
Array2D_Vector3 NURBS::NURBSRectangle<Real>::simpleRemove( int idx, int dir )
{
	int nU = mNumUCtrlPoints;
	int nV = mNumVCtrlPoints;

	Array2D_Vector3 Qw( nU - (dir == 0 ? 1 : 0), Array1D_Vector3(nV - (dir == 1 ? 1 : 0), Vector3(0,0,0)) );

	int REMOVE_ROW = (dir == 0 ? idx : -1);
	int REMOVE_COLUMN = (dir == 1 ? idx : -1);

	int p = 0;
	for( int i = 0; i < nU; ++i){
		if (i == REMOVE_ROW) continue;

		int q = 0;
		for( int j = 0; j < nV; ++j){
			if (j == REMOVE_COLUMN)	continue;

			Qw[p][q] = mCtrlPoint[i][j];
			++q;
		}

		++p;
	}

	return Qw;
}


//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
//template
//class NURBSRectangle<float>;

template
class NURBSRectangle<double>;
//----------------------------------------------------------------------------
}
