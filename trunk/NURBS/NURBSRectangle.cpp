#include "NURBSRectangle.h"
#include "NURBSCurve.h"

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

	Vector3 corner = center - (dU * width * 0.5) - (dV * length * 0.5);

	Scalar aspect_U = 1.0, aspect_V = 1.0;
	if(width > length) aspect_U = length / width;
	if(length > width) aspect_V = width / length;

	nU *= 1.0 / aspect_U;
	nV *= 1.0 / aspect_V;

    // Rectangular surface
    std::vector< std::vector<Vec3d> > pts( nU, std::vector<Vec3d>( nV, Vector3(0) ) );
    std::vector< std::vector<Scalar> > weights( nU, std::vector<Scalar>( nV, 1.0 ) );

    Vector3 deltaU = (width  / (nU-1)) * dU;
    Vector3 deltaV = (length / (nV-1)) * dV;

    for(int y = 0; y < nV; y++){
        for(int x = 0; x < nU; x++)
		{
            pts[x][y] = corner + (deltaU * x) + (deltaV * y);
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

void NURBSRectangle::SetControlPoint (int uIndex, int vIndex, Vector3& ctrl)
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

Vector3 NURBSRectangle::GetControlPoint (int uIndex, int vIndex)
{
    if (0 <= uIndex && uIndex < mNumUCtrlPoints
    &&  0 <= vIndex && vIndex < mNumVCtrlPoints)
    {
        return mCtrlPoint[uIndex][vIndex];
    }

    return Vector3(MAX_REAL, MAX_REAL, MAX_REAL);
}

//----------------------------------------------------------------------------

std::vector<Vector3> NURBSRectangle::GetControlPointsV( int vIndex )
{
	return mCtrlPoint[vIndex];
}

std::vector<Vector3> NURBSRectangle::GetControlPointsU( int uIndex )
{
	std::vector<Vector3> pts;

	for(int i = 0; i < (int)mCtrlPoint.size(); i++)
		pts.push_back(mCtrlPoint[i][uIndex]);

	return pts;
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
	std::vector<Real> valU,valV;
	uniformCoordinates(valU, valV, resolution);

	for(int y = 0; y < (int)valV.size() - 1; y++)
	{
		for(int x = 0; x < (int)valU.size() - 1; x++)
		{
			std::vector<Vector3> pos = std::vector<Vec3d>(4, Vector3(0));
			std::vector<Vector3> dU = pos, dV = pos, normal = pos;

			GetFrame(valU[x]	, valV[y]	,	pos[0], dU[0], dV[0], normal[0]);
			GetFrame(valU[x+1]	, valV[y]	,	pos[1], dU[1], dV[1], normal[1]);
			GetFrame(valU[x+1]	, valV[y+1]	,	pos[2], dU[2], dV[2], normal[2]);
			GetFrame(valU[x]	, valV[y+1]	,	pos[3], dU[3], dV[3], normal[3]);

			SurfaceQuad quad;

			for(int k = 0; k < 4; k++){
				quad.p[k] = pos[k];
				quad.n[k] = normal[k];
			}

			quads.push_back(quad);
		}
    }

}

std::vector< std::vector<Vector3> > NURBSRectangle::generateSurfaceTris( Scalar resolution )
{
	std::vector< std::vector<Vector3> > tris;

	std::vector<Vector3> ctrlU = GetControlPointsU(0);
	std::vector<Vector3> ctrlV = GetControlPointsV(0);

	std::vector< std::vector<Vector3> > curveU = NURBSCurve(ctrlU, std::vector<Scalar>(ctrlU.size(), 1.0)).toSegments(resolution);
	std::vector< std::vector<Vector3> > curveV = NURBSCurve(ctrlV, std::vector<Scalar>(ctrlV.size(), 1.0)).toSegments(resolution);

	std::vector<Real> valU,valV;
	uniformCoordinates(valU, valV, qMin(curveU.size(), curveV.size()));

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

void NURBSRectangle::generateSurfacePoints( Scalar stepSize, std::vector< std::vector<Vector3> > & points, std::vector<Real> & valU, std::vector<Real> & valV )
{
	std::vector<Vector3> ctrlU = GetControlPointsU(0);
	std::vector<Vector3> ctrlV = GetControlPointsV(0);

	std::vector< std::vector<Vector3> > curveU = NURBSCurve(ctrlU, std::vector<Scalar>(ctrlU.size(), 1.0)).toSegments(stepSize);
	std::vector< std::vector<Vector3> > curveV = NURBSCurve(ctrlV, std::vector<Scalar>(ctrlV.size(), 1.0)).toSegments(stepSize);

	uniformCoordinates(valU, valV, qMin(curveU.size(), curveV.size()));

	points.resize(valU.size(), std::vector<Vector3>(valV.size()));

	for(int y = 0; y < (int)valV.size(); y++)
	{
		for(int x = 0; x < (int)valU.size(); x++)
		{
			points[x][y] = P(valU[x], valV[y]);
		}
	}
}
//----------------------------------------------------------------------------

Vec2d NURBSRectangle::timeAt( const Vector3 & pos )
{
	std::vector< std::vector<Vector3> > pts;

	std::vector<Real> valU, valV;

	Scalar stepSize = 0.1;

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
	Vec2d minRange( valU[qMax(0, minIdxU - 1)], valV[qMax(0, minIdxV - 1)]);
	Vec2d maxRange( valU[qMin((int)valU.size() - 1, minIdxU + 1)], valV[qMin((int)valV.size() - 1, minIdxV + 1)] );

	Vec2d bestUV( valU[minIdxU], valV[minIdxV] );

	return timeAt(pos, bestUV, minRange, maxRange, minDist);
}

Vec2d NURBSRectangle::timeAt( const Vector3 & pos, Vec2d & bestUV, Vec2d & minRange, Vec2d & maxRange, Real currentDist, Real threshold )
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
		return Vec2d(curU, curV);
	else
	{
		// 4) Otherwise recursive search in smaller range
		Vec2d minRange( qMax(0.0, curU - du), qMax(0.0, curV - dv));
		Vec2d maxRange( qMin(1.0, curU + du), qMin(1.0, curV + dv) );

		return timeAt(pos, Vec2d(curU, curV), minRange, maxRange, minDist, threshold);
	}
}

Real NURBSRectangle::aspectU()
{
	Real width = this->GetNumCtrlPoints(0);
	Real length = this->GetNumCtrlPoints(1);

	Scalar aspect = 1.0;
	if(length > width) aspect = length / width;
	return aspect;
}

Real NURBSRectangle::aspectV()
{
	Real width = this->GetNumCtrlPoints(0);
	Real length = this->GetNumCtrlPoints(1);

	Scalar aspect = 1.0;
	if(width > length) aspect = width / length;
	return aspect;
}

void NURBSRectangle::uniformCoordinates( std::vector<Real> & valU, std::vector<Real> & valV, int resolution, int u, int v )
{
	Scalar resU = resolution * aspectV();
	Scalar resV = resolution * aspectU();

	// Uniform across given U, V
	std::vector<Vector3> cptsU = GetControlPointsU(u);
	std::vector<Vector3> cptsV = GetControlPointsV(v);
	NURBSCurve curveU( cptsU, std::vector<Scalar>(cptsU.size(), 1) );
	NURBSCurve curveV( cptsV, std::vector<Scalar>(cptsV.size(), 1) );
	Scalar deltaU = curveU.GetLength(0, 1) / resU;
	Scalar deltaV = curveV.GetLength(0, 1) / resV;
	for(int i = 0; i <= resU; i++)	valU.push_back(curveU.GetTime(i * deltaU));
	for(int i = 0; i <= resV; i++)	valV.push_back(curveV.GetTime(i * deltaV));
}

Real NURBSRectangle::avgCtrlEdgeLength()
{
	Real sum = 0;
	int c = 0;

	for(int y = 0; y < (int)mCtrlPoint.size() - 1; y++){
		for(int x = 0; x < (int)mCtrlPoint[0].size() - 1; x++){
			sum += (mCtrlPoint[y][x] - mCtrlPoint[y+1][x+1] ).norm();
			c++;
		}
	}

	return sum / c;
}
