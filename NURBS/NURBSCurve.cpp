#include "NURBSCurve.h"
#include "LineSegment.h"
using namespace NURBS;

NURBSCurve::NURBSCurve (std::vector<Vector3> ctrlPoint, std::vector<Real> ctrlWeight,
                        int degree, bool loop, bool open) : SingleCurve((Real)0, (Real)1), mLoop(loop)
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

NURBSCurve NURBSCurve::createCurve( Vector3 from, Vector3 to, int steps )
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

void NURBSCurve::CreateControl (std::vector<Vector3> ctrlPoint, std::vector<Real> ctrlWeight)
{
	int newNumCtrlPoints = mNumCtrlPoints + mReplicate;
    newNumCtrlPoints = newNumCtrlPoints;

	mCtrlPoint = ctrlPoint;
	mCtrlWeight = ctrlWeight;
}

//----------------------------------------------------------------------------

int NURBSCurve::GetNumCtrlPoints ()
{
    return mNumCtrlPoints;
}
//----------------------------------------------------------------------------

int NURBSCurve::GetDegree ()
{
    return mBasis.GetDegree();
}
//----------------------------------------------------------------------------

bool NURBSCurve::IsOpen ()
{
    return mBasis.IsOpen();
}
//----------------------------------------------------------------------------

bool NURBSCurve::IsUniform ()
{
    return mBasis.IsUniform();
}
//----------------------------------------------------------------------------

bool NURBSCurve::IsLoop ()
{
    return mLoop;
}
//----------------------------------------------------------------------------

void NURBSCurve::SetControlPoint (int i,  Vector3& ctrl)
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

Vector3 NURBSCurve::GetControlPoint (int i)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlPoint[i];
    }

    return Vector3(MAX_REAL, MAX_REAL, MAX_REAL);
}
//----------------------------------------------------------------------------

std::vector<Vector3> NURBSCurve::getControlPoints()
{
	return mCtrlPoint;
}
//----------------------------------------------------------------------------

std::vector<Real> NURBSCurve::getControlWeights()
{
	return mCtrlWeight;
}
//----------------------------------------------------------------------------

void NURBSCurve::SetControlWeight (int i, Real weight)
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

Real NURBSCurve::GetControlWeight (int i)
{
    if (0 <= i && i < mNumCtrlPoints)
    {
        return mCtrlWeight[i];
    }

    return MAX_REAL;
}
//----------------------------------------------------------------------------

void NURBSCurve::SetKnot (int i, Real knot)
{
    mBasis.SetKnot(i, knot);
}
//----------------------------------------------------------------------------

Real NURBSCurve::GetKnot (int i)
{
    return mBasis.GetKnot(i);
}
//----------------------------------------------------------------------------

std::vector < std::vector<Vector3> > NURBSCurve::toSegments( Scalar resolution )
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

void NURBSCurve::Get (Real t, Vector3* pos,
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

BSplineBasis& NURBSCurve::GetBasis ()
{
    return mBasis;
}
//----------------------------------------------------------------------------

Vector3 NURBSCurve::GetPosition (Real t)
{
    Vector3 pos;
    Get(t, &pos, 0, 0, 0);
    return pos;
}
//----------------------------------------------------------------------------

Vector3 NURBSCurve::GetFirstDerivative (Real t)
{
    Vector3 der1;
    Get(t, 0, &der1, 0, 0);
    return der1;
}
//----------------------------------------------------------------------------

Vector3 NURBSCurve::GetSecondDerivative (Real t)
{
    Vector3 der2;
    Get(t, 0, 0, &der2, 0);
    return der2;
}
//----------------------------------------------------------------------------

Vector3 NURBSCurve::GetThirdDerivative (Real t)
{
    Vector3 der3;
    Get(t, 0, 0, 0, &der3);
    return der3;
}
//----------------------------------------------------------------------------

Real NURBSCurve::timeAt( const Vector3 & pos )
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

void NURBSCurve::translate( const Vector3 & delta )
{
	for(int i = 0; i < (int)mCtrlPoint.size(); i++)
		mCtrlPoint[i] += delta;
}

void NURBSCurve::scale( Scalar scaleFactor )
{
	for(int i = 0; i < (int)mCtrlPoint.size(); i++)
		mCtrlPoint[i] *= scaleFactor;
}

void NURBSCurve::scaleInPlace( Scalar scaleFactor, int placeCtrlPoint )
{
	Vector3 delta = mCtrlPoint[placeCtrlPoint];
	translate( -delta );
	scale(scaleFactor);
	translate( delta );
}

void NURBSCurve::translateTo( const Vector3 & newPos, int cpIDX )
{
	Vec3d cp = mCtrlPoint[cpIDX];
	Vec3d delta = newPos - cp;
	for(int i = 0; i < GetNumCtrlPoints(); i++)
		mCtrlPoint[i] += delta;
}

static inline Vec3d rotatedVec(const Vec3d & v, double theta, const Vec3d & axis)
{
	return (v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)));
}

void NURBSCurve::bend( double amount )
{
	Vec3d axis = GetBinormal(0);
	double theta = amount * 3.14;

	for(int j = 1; j < (int)mCtrlPoint.size(); j++)
	{
		double angle = theta * j-1;

		for(int i = j; i < (int)mCtrlPoint.size(); i++)
		{
			mCtrlPoint[i] = mCtrlPoint[i-1] + rotatedVec(mCtrlPoint[i] - mCtrlPoint[i-1], angle, axis);
		}
	}
}
