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
template <typename Real>
std::vector<Vector3> NURBSCurve<Real>::getControlPoints()
{
    return mCtrlPoint;
}

template <typename Real>
Real NURBSCurve<Real>::timeAt( const Vector3 & pos )
{
	int segmentCount = 100;

	std::vector<Real> times;
	SubdivideByLengthTime(segmentCount, times);

    int minIdx = 0;
    double t = 0.0;
    Vector3 d(0);
    Scalar minDist = std::numeric_limits<Scalar>::max();

    for(int i = 0; i < ((int)times.size()) - 1; i++)
    {
        Line segment(this->GetPosition(times[i]), this->GetPosition(times[i+1]));
        segment.ClosestPoint(pos, t, d);

        Scalar dist = (pos - d).norm();

        if(dist < minDist){
            minDist = dist;
            minIdx = i;
        }
    }

	Line closestSegment(this->GetPosition(times[minIdx]), this->GetPosition(times[minIdx+1]));

    closestSegment.ClosestPoint(pos, t, d);
    return qRanged(0.0, ((1-t) * times[minIdx]) + (t * times[minIdx+1]), 1.0);
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

template <typename Real>
Array1D_Real NURBSCurve<Real>::GetKnotVector(bool isInnerOnly)
{
	if(isInnerOnly)
	{
		int d = this->GetDegree();

		Array1D_Real result(mBasis.mKnot.begin() + d, mBasis.mKnot.end() - d);
		
		return result;
	}
	else
		return this->mBasis.mKnot;
}

template <typename Real>
int NURBSCurve<Real>::findSpan(int n, int p, Real u, Array1D_Real U)
{
	int low, high, mid;

	if (u == U[n+1]) return(n);

	low = p;
	high = n+1;
	mid = (int)(low+high)/2;
	while (u < U[mid] || u >= U[mid+1]) 
	{
		if (u < U[mid]) 
			high = mid;
		else low = mid;
		mid = (int)(low+high)/2;
	}
	return mid;
}

template <typename Real>
int NURBSCurve<Real>::findSpan(Real u) 
{
	return findSpan(this->mCtrlPoint.size() - 1, GetDegree(), u, GetKnotVector() );
}

template <typename Real>
Array1D_Vector3 NURBS::NURBSCurve<Real>::midPointRefined()
{
	Array1D_Vector3 newCtrlPoints;
	Array1D_Real Ubar;
	computeMidPointRefine(newCtrlPoints, Ubar);
	return newCtrlPoints;
}

template <typename Real>
void NURBSCurve<Real>::computeMidPointRefine( Array1D_Vector3 & Qw, Array1D_Real & Ubar )
{
	std::vector<Real> insknts;

	Array1D_Real oldKnotVec = GetKnotVector(true);

	for(int i = 0; i < (int)oldKnotVec.size() - 1; i++){
		double range = (oldKnotVec[i+1] - oldKnotVec[i]);
		insknts.push_back( (0.5 * range) + oldKnotVec[i] );
	}

	refine(insknts, Qw, Ubar);
}

template <typename Real>
void NURBSCurve<Real>::refine(Array1D_Real & insknts, Array1D_Vector3 & Qw, Array1D_Real & Ubar )
{
	// Input
	Array1D_Real X		= insknts;
	Array1D_Vector3 Pw	= this->mCtrlPoint;
	Array1D_Real U		= GetKnotVector();

	int s = insknts.size();
	int order = this->GetDegree() + 1;
	int p = order - 1;
	int n = Pw.size() - 1;
	int r = X.size() - 1;
	int m = n+p+1;

	// Output
	Qw.resize	( n+s+1, Vector3(0) );
	Ubar.resize	( m+s+1, 0 );

	int a = findSpan(n,p,X[0],U);
	int b = findSpan(n,p,X[r],U) + 1;
	
	int j = 0;
	for(j=0	 ; j<=a-p; j++)  Qw[j]		 = Pw[j];
	for(j=b-1; j<=n	 ; j++)  Qw[j+r+1]	 = Pw[j];
	for(j=0	 ; j<=a	 ; j++)  Ubar[j]	 = U[j];
	for(j=b+p; j<=m	 ; j++)  Ubar[j+r+1] = U[j];

	int i = b+p-1;
	int k = b+p+r;

	Real alfa = 0;

	for (int j=r; j>=0; j--) 
	{
		while (X[j] <= U[i] && i > a) {
			Qw[k-p-1] = Pw[i-p-1];
			Ubar[k] = U[i];
			k = k - 1;
			i = i - 1;
		}

		Qw[k-p-1] = Qw[k-p];

		for (int l=1; l<=p; l++) 
		{
			int ind = k-p+l;
			alfa = Ubar[k+l] - X[j];
			if (std::abs(alfa) == 0.0)
			{
				Qw[ind-1] = Qw[ind];
			}
			else 
			{
				alfa = alfa / (Ubar[k+l] - U[i-p+l]);
				Qw[ind-1] = alfa * Qw[ind-1] + (1.0 - alfa) * Qw[ind];
			}
		}
		Ubar[k] = X[j];
		k = k - 1;
	}
}

/*	@param u           Parametric coordinate of breaking point
	@param k           Knot index of insertion point
	@param s           Knot multiplicity at k
	@param r           Number of times a knot should be inserted */
template <typename Real>
Array1D_Real NURBSCurve<Real>::insertKnot(Real u, int k, int s, int r, Array1D_Vector3 & Qw, int * uq)
{
	Array1D_Real	UP = GetKnotVector();
	Array1D_Vector3 Pw = mCtrlPoint;
	Array1D_Real	UQ;

	Array1D_Vector3 Rw;
	Scalar alpha;
	int L = 0;

	int order = this->GetDegree() + 1;
	int p = order - 1;
	int np = Pw.size() - 1;

	int mp = np+p+1;
	int nq = np+r;

	// Define array
	UQ.resize(mp+r+1);
	Qw.resize(nq+1);
	Rw.resize(p+1);

	// Load new knot vector
	for (int i=0; i<=k; i++)	UQ[i]	= UP[i];
	for (int i=1; i<=r; i++)	UQ[k+i] = u;
	for (int i=k+1; i<=mp; i++)	UQ[i+r] = UP[i];

	// Save unaltered control points
	for (int i=0; i<=k-p; i++)	Qw[i]	= Pw[i];
	for (int i=k-s; i<=np; i++)	Qw[i+r] = Pw[i];
	for (int i=0; i<=p-s; i++)	Rw[i]	= Pw[k-p+i];

	// Insert the knot r times
	for (int j=1; j<=r; j++) 
	{
		L = k-p+j;
		for (int i=0; i<=p-j-s; i++) 
		{
			alpha = (u-UP[L+i])/(UP[i+k+1]-UP[L+i]);
			Rw[i] = alpha*Rw[i+1] + (1.0 - alpha) * Rw[i];
		}
		Qw[L] = Rw[0];
		Qw[k+r-j-s] = Rw[p-j-s];
	}

	// Load remaining control points
	for (int i=L+1; i<k-s; i++)
		Qw[i] = Rw[i-L];

	if(uq) *uq = k-p+1;

	return UQ;
}

template <typename Real>
Array1D_Vector3 NURBSCurve<Real>::simpleRefine( int k )
{
	if(k <= 0) return this->mCtrlPoint;

	NURBSCurve<Real> curCurve = *this;

	for(int i = 0; i < k ; i++)
	{
		int segment = 0;
		double maxDist = -DBL_MAX;

		// Find longest segment
		for(int j = 0; j < (int)(curCurve.mCtrlPoint.size() - 1); j++)
		{
			double dist = (curCurve.mCtrlPoint[j+1] - curCurve.mCtrlPoint[j]).norm();

			if(dist > maxDist){
				maxDist = dist;
				segment = j;
			}
		}

		// Insert knot at middle of longest segment
		Array1D_Real U = curCurve.GetKnotVector(true);

		double t = double(segment) / (curCurve.mCtrlPoint.size()-2);

		int idx = t * (U.size()-2);
		double range = U[idx+1] - U[idx];
		double u = (0.5 * range) + U[idx];

		Array1D_Vector3 newPnts;
		Array1D_Real Ubar;
        Array1D_Real insk = Array1D_Real(1,u);
        curCurve.refine(insk, newPnts, Ubar);

		// Update control points
		curCurve = NURBSCurve<Real>::createCurveFromPoints(newPnts);
	}

	return Array1D_Vector3(curCurve.mCtrlPoint.begin(),curCurve.mCtrlPoint.end());
}

template <typename Real>
Real NURBSCurve<Real>::KnotRemovalError( int r, int s )
{
	Array1D_Real	U = GetKnotVector();
	Array1D_Vector3 P = mCtrlPoint;

	Array1D_Vector3 temp(P.size(), Vector3(0.0)) ;
	
	int deg_ = GetDegree();

	int ord = deg_+1 ;
	int last = r-s ;
	int first = r-deg_ ;
	int off ; 
	int i,j,ii,jj ;
	Real alfi,alfj ;
	Real u ;

	u = U[r] ;

	off = first-1;
	temp[0] = P[off] ;
	temp[last+1-off] = P[last+1] ;

	i=first ; j=last ;
	ii=1 ; jj=last-off ;

	while(j-i>0){
		alfi = (u-U[i])/(U[i+ord]-U[i]) ;
		alfj = (u-U[j])/(U[j+ord]-U[j]) ;
		temp[ii] = (P[i]-(1.0-alfi)*temp[ii-1])/alfi ; 
		temp[jj] = (P[j]-alfj*temp[jj+1])/(1.0-alfj) ;
		++i ; ++ii ;
		--j ; --jj ;
	}
	if(j-i<0){
		return (temp[ii-1]-temp[jj+1]).norm() ;
	}
	else{
		alfi=(u-U[i])/(U[i+ord]-U[i]) ;
		return (P[i]-alfi*temp[ii+1]+(1.0-alfi)*temp[ii-1]).norm() ;
	}

}

template <typename Real>
Array1D_Vector3 NURBSCurve<Real>::removeKnots(int iterations)
{
	NURBSCurve<Real> curCurve = *this;

	for(int itr = 0; itr < iterations; itr++)
	{
		Array1D_Real	U = curCurve.GetKnotVector();
		Array1D_Vector3 P = curCurve.mCtrlPoint;
		int p = curCurve.GetDegree();

		QMap<Real, int> errors;
		for(int i = p + 1; i < (int)U.size() - 1; i++)
		{
			if(U[i] < U[i+1])
				errors[ curCurve.KnotRemovalError(i, 1) ] = i;
		}

		int r = errors.values().at(itr); // Knot to remove

		int num = 1;
		int s = 1;

		int deg_ = curCurve.GetDegree();
		int m = U.size() ;
		int ord = deg_+1 ;
		int fout = (2*r-s-deg_)/2 ;
		int last = r-s ;
		int first = r-deg_ ;
		Real alfi, alfj ;
		int i,j,k,ii,jj,off ;
		Real u ;

		Array1D_Vector3 temp( 2*deg_+1, Vector3(0) ) ;

		u = U[r] ;

		int t;
		for(t=0;t<num;++t)
		{
			off = first-1 ;
			temp[0] = P[off] ;
			temp[last+1-off] = P[last+1] ;
			i = first; j = last ;
			ii = 1 ; jj = last-off ;
			while(j-i > t){
				alfi = (u-U[i])/(U[i+ord+t]-U[i]) ;
				alfj = (u-U[j-t])/(U[j+ord]-U[j-t]) ;
				temp[ii] = (P[i]-(1.0-alfi)*temp[ii-1])/alfi ;
				temp[jj] = (P[j]-alfj*temp[jj+1])/(1.0-alfj) ;
				++i ; ++ii ;
				--j ; --jj ;
			}
			i = first ; j = last ;
			while(j-i>t){
				P[i] = temp[i-off] ;
				P[j] = temp[j-off] ;
				++i; --j ;
			}
			--first ; ++last ;
		}
		if(t == 0)
			return P;

		for(k=r+1; k<m ; ++k)
			U[k-t] = U[k] ;
		j = fout ; i=j ; // Pj thru Pi will be overwritten
		for(k=1; k<t; k++)
			if( (k%2) == 1)
				++i ;
			else
				--j ;
		for(k=i+1; k<(int)P.size() ; k++) {// Shift
			P[j++] = P[k] ; 
		}

		P.resize(P.size() - t);
		curCurve = createCurveFromPoints(P);
	}

	return curCurve.mCtrlPoint;
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
