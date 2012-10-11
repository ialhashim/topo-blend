#include "NURBSRectangle.h"

//----------------------------------------------------------------------------
NURBSRectangle::NURBSRectangle (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
    int uDegree, int vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen) : ParametricSurface(0, 1, 0, 1, true)
{
    int numUCtrlPoints = ctrlPoint.size();
    int numVCtrlPoints = ctrlPoint[0].size();

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
NURBSRectangle NURBSRectangle::createSheet(Scalar width, Scalar length, Vector3 center, Vector3 dU, Vector3 dV)
{
    int nU = 4, nV = 4;
    int degree = 3;

    // Rectangular surface
    std::vector< std::vector<Vec3d> > pts( nV, std::vector<Vec3d>( nU ) );
    std::vector< std::vector<Scalar> > weights( nV, std::vector<Scalar>( nU, 1.0 ) );

    Vector3 deltaU = (width  / (nU-1)) * dU;
    Vector3 deltaV = (length / (nV-1)) * dV;

    Vector3 corner = center - (dU * width * 0.5) - (dV * length * 0.5);

	Vector3 n = cross(dU,dV);

	Vector3 cU = (dU * width * 0.5);
	Vector3 cV = (dV * length * 0.5);

    for(int y = 0; y < nV; y++){
        for(int x = 0; x < nU; x++){
            pts[y][x] = corner + (deltaU * x) + (deltaV * y);
        }
    }

    return NURBSRectangle(pts, weights, degree, degree, false, false, true, true);
}

//----------------------------------------------------------------------------
void NURBSRectangle::CreateControl (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight)
{
    int newNumUCtrlPoints = mNumUCtrlPoints + mUReplicate;
    int newNumVCtrlPoints = mNumVCtrlPoints + mVReplicate;

    newNumUCtrlPoints = newNumUCtrlPoints;
    newNumVCtrlPoints = newNumVCtrlPoints;

    mCtrlPoint = ctrlPoint;
    mCtrlWeight = ctrlWeight;
}
//----------------------------------------------------------------------------

int NURBSRectangle::GetNumCtrlPoints (int dim)
{
    return mBasis[dim].GetNumCtrlPoints();
}
//----------------------------------------------------------------------------

int NURBSRectangle::GetDegree (int dim)
{
    return mBasis[dim].GetDegree();
}
//----------------------------------------------------------------------------

bool NURBSRectangle::IsOpen (int dim)
{
    return mBasis[dim].IsOpen();
}
//----------------------------------------------------------------------------

bool NURBSRectangle::IsUniform (int dim)
{
    return mBasis[dim].IsUniform();
}
//----------------------------------------------------------------------------

bool NURBSRectangle::IsLoop (int dim)
{
    return mLoop[dim];
}
//----------------------------------------------------------------------------

void NURBSRectangle::SetControlPoint (int uIndex, int vIndex,
     Vector3& ctrl)
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

Vector3 NURBSRectangle::GetControlPoint (int uIndex,
    int vIndex)
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        return mCtrlPoint[uIndex][vIndex];
    }

    return Vector3(MAX_REAL, MAX_REAL, MAX_REAL);
}
//----------------------------------------------------------------------------

void NURBSRectangle::SetControlWeight (int uIndex, int vIndex,
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

Real NURBSRectangle::GetControlWeight (int uIndex, int vIndex)
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        return mCtrlWeight[uIndex][vIndex];
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------

void NURBSRectangle::SetKnot (int dim, int i, Real knot)
{
    if (0 <= dim && dim <= 1)
    {
        mBasis[dim].SetKnot(i,knot);
    }
}
//----------------------------------------------------------------------------

Real NURBSRectangle::GetKnot (int dim, int i)
{
    if (0 <= dim && dim <= 1)
    {
        return mBasis[dim].GetKnot(i);
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------

void NURBSRectangle::Get (Real u, Real v, Vector3& pos,
    Vector3& derU, Vector3& derV, Vector3& derUU,Vector3& derUV, Vector3& derVV)
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
            tmp = mBasis[0].GetD0(iu)*mBasis[1].GetD0(iv) * mCtrlWeight[iu][iv];
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
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD0(iv) * mCtrlWeight[iu][iv];
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
                tmp = mBasis[0].GetD2(iu)*mBasis[1].GetD0(iv) * mCtrlWeight[iu][iv];
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
                tmp = mBasis[0].GetD1(iu)*mBasis[1].GetD1(iv) * mCtrlWeight[iu][iv];
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

Vector3 NURBSRectangle::P (Real u, Real v)
{
    Vector3 pos;
    Get(u, v, pos);
    return pos;
}
//----------------------------------------------------------------------------

Vector3 NURBSRectangle::PU (Real u, Real v)
{
    Vector3 derU;
    Get(u, v, Vector3(0), derU);
    return derU;
}
//----------------------------------------------------------------------------

Vector3 NURBSRectangle::PV (Real u, Real v)
{
    Vector3 derV;
    Get(u, v, Vector3(0), Vector3(0), derV);
    return derV;
}
//----------------------------------------------------------------------------

Vector3 NURBSRectangle::PUU (Real u, Real v)
{
    Vector3 derUU;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), derUU);
    return derUU;
}
//----------------------------------------------------------------------------

Vector3 NURBSRectangle::PUV (Real u, Real v)
{
    Vector3 derUV;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), Vector3(0), derUV);
    return derUV;
}
//----------------------------------------------------------------------------

Vector3 NURBSRectangle::PVV (Real u, Real v)
{
    Vector3 derVV;
    Get(u, v, Vector3(0), Vector3(0), Vector3(0), Vector3(0), Vector3(0), derVV);
    return derVV;
}
//----------------------------------------------------------------------------

void NURBSRectangle::generateSurfaceQuads( int resolution )
{
	double res = resolution;
	double du = 1.0 / res;
	double dv = 1.0 / res;

	for(int y = 0; y < res; y++){
		double v = double(y) / res;

		for(int x = 0; x < res; x++){
			double u = double(x) / res;

			std::vector<Vec3d> pos = std::vector<Vec3d>(4, Vec3d(0));
			std::vector<Vec3d> dU = pos, dV = pos, normal = pos;

			GetFrame(u		,v		,	pos[0], dU[0], dV[0], normal[0]);
			GetFrame(u+du	,v		,	pos[1], dU[1], dV[1], normal[1]);
			GetFrame(u+du	,v+dv	,	pos[2], dU[2], dV[2], normal[2]);
			GetFrame(u		,v+dv	,	pos[3], dU[3], dV[3], normal[3]);

			SurfaceQuad quad;

			for(int k = 0; k < 4; k++){
				quad.p[k] = pos[k];
				quad.n[k] = normal[k];
			}

			quads.push_back(quad);
		}
    }
}

void NURBSRectangle::generateSurfacePoints( double stepSize, std::vector< std::vector<Vector3> > & points )
{
	int cU = 1 + (1.0 / stepSize);
	int cV = 1 + (1.0 / stepSize);

	points.resize( cU, std::vector<Vector3>( cV, Vector3(0)) );

	int i = 0, j = 0;
	for(double v = 0; v <= 1; v += stepSize)
	{
		for(double u = 0; u <= 1; u += stepSize)
			points[j][i++] = P(u,v);

		// Indexing
		j++; i = 0;
	}
}
//----------------------------------------------------------------------------

Vec2d NURBSRectangle::timeAt( const Vector3 & pos )
{
	double stepSize = 0.1;

	std::vector< std::vector<Vector3> > pts;

	generateSurfacePoints(stepSize, pts);

	int cU = 1 + (1.0 / stepSize);
	int cV = 1 + (1.0 / stepSize);

	int minIdxU = 0, minIdxV = 0;
	Scalar minDist = std::numeric_limits<Scalar>::max();

	for(int v = 0; v < cV; v++){
		for(int u = 0; u < cU; u++){
			Scalar dist = (pts[v][u] - pos).norm();

			if(dist < minDist){
				minDist = dist;
				minIdxU = u;
				minIdxV = v;
			}
		}
	}

	// More accurate search
	double minU = minIdxU * stepSize, minV = minIdxV * stepSize;

	return Vec2d(minU, minV);
}
