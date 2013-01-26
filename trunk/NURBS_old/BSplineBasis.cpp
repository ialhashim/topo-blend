#include "BSplineBasis.h"

//----------------------------------------------------------------------------
BSplineBasis::BSplineBasis ()
{
}
//----------------------------------------------------------------------------
BSplineBasis::BSplineBasis (int numCtrlPoints, int degree, bool open)
{
    Create(numCtrlPoints, degree, open);
}
//----------------------------------------------------------------------------
void BSplineBasis::Create (int numCtrlPoints, int degree, bool open)
{
    mUniform = true;

    int i, numKnots = Initialize(numCtrlPoints, degree, open);
    Real factor = ((Real)1)/(mNumCtrlPoints - mDegree);
    if (mOpen)
    {
        for (i = 0; i <= mDegree; ++i)
        {
            mKnot[i] = (Real)0;
        }

        for (/**/; i < mNumCtrlPoints; ++i)
        {
            mKnot[i] = (i - mDegree)*factor;
        }

        for (/**/; i < numKnots; ++i)
        {
            mKnot[i] = (Real)1;
        }
    }
    else
    {
        for (i = 0; i < numKnots; ++i)
        {
            mKnot[i] = (i - mDegree)*factor;
        }
    }
}
//----------------------------------------------------------------------------

BSplineBasis::BSplineBasis (int numCtrlPoints, int degree,
    const Real* interiorKnot)
{
    Create(numCtrlPoints, degree, interiorKnot);
}
//----------------------------------------------------------------------------

void BSplineBasis::Create (int numCtrlPoints, int degree,
    const Real* interiorKnot)
{
    mUniform = false;

    int i, numKnots = Initialize(numCtrlPoints, degree, true);
    for (i = 0; i <= mDegree; ++i)
    {
        mKnot[i] = (Real)0;
    }

    for (int j = 0; i < mNumCtrlPoints; ++i, ++j)
    {
        mKnot[i] = interiorKnot[j];
    }

    for (/**/; i < numKnots; ++i)
    {
        mKnot[i] = (Real)1;
    }
}
//----------------------------------------------------------------------------

int BSplineBasis::GetNumCtrlPoints () const
{
    return mNumCtrlPoints;
}
//----------------------------------------------------------------------------

int BSplineBasis::GetDegree () const
{
    return mDegree;
}
//----------------------------------------------------------------------------

bool BSplineBasis::IsOpen () const
{
    return mOpen;
}
//----------------------------------------------------------------------------

bool BSplineBasis::IsUniform () const
{
    return mUniform;
}
//----------------------------------------------------------------------------

Real BSplineBasis::GetD0 (int i) const
{
    return mBD0[mDegree][i];
}
//----------------------------------------------------------------------------

Real BSplineBasis::GetD1 (int i) const
{
    return mBD1[mDegree][i];
}
//----------------------------------------------------------------------------

Real BSplineBasis::GetD2 (int i) const
{
    return mBD2[mDegree][i];
}
//----------------------------------------------------------------------------

Real BSplineBasis::GetD3 (int i) const
{
    return mBD3[mDegree][i];
}
//----------------------------------------------------------------------------

Array2D_Real BSplineBasis::Allocate()
{
    int numRows = mDegree + 1;
    int numCols = mNumCtrlPoints + mDegree;

    Array2D_Real data = Array2D_Real( numRows, Array1D_Real( numCols, 0.0 ) );

    return data;
}
//----------------------------------------------------------------------------

int BSplineBasis::Initialize (int numCtrlPoints, int degree, bool open)
{
    assert(numCtrlPoints >= 2);
    assert(1 <= degree && degree <= numCtrlPoints-1);

    mNumCtrlPoints = numCtrlPoints;
    mDegree = degree;
    mOpen = open;

    int numKnots = mNumCtrlPoints + mDegree + 1;
    mKnot = Array1D_Real(numKnots, 0);

    mBD0 = Allocate();
    mBD1.clear();
    mBD2.clear();
    mBD3.clear();

    return numKnots;
}
//----------------------------------------------------------------------------

void BSplineBasis::SetKnot (int j, Real value)
{
    if (!mUniform)
    {
        // Access only allowed to elements d+1 <= i <= n.
        int i = j + mDegree + 1;
        if (mDegree + 1 <= i && i <= mNumCtrlPoints)
        {
            mKnot[i] = value;
        }
        else
        {
            assert(false);
        }
    }
    else
    {
        assert(false);
    }
}
//----------------------------------------------------------------------------

Real BSplineBasis::GetKnot (int j) const
{
    // Access only allowed to elements d+1 <= i <= n.
    int i = j + mDegree + 1;
    if (mDegree + 1 <= i && i <= mNumCtrlPoints)
    {
        return mKnot[i];
    }

    assert(false);
    return MAX_REAL;
}
//----------------------------------------------------------------------------

int BSplineBasis::GetKey (Real& t) const
{
    if (mOpen)
    {
        // Open splines clamp to [0,1].
        if (t <= (Real)0)
        {
            t = (Real)0;
            return mDegree;
        }
        else if (t >= (Real)1)
        {
            t = (Real)1;
            return mNumCtrlPoints - 1;
        }
    }
    else
    {
        // Periodic splines wrap to [0,1).
        if (t < (Real)0 || t >= (Real)1)
        {
            t -= floor(t);
        }
    }


    int i;

    if (mUniform)
    {
        i = mDegree + (int)((mNumCtrlPoints - mDegree)*t);
    }
    else
    {
        for (i = mDegree + 1; i <= mNumCtrlPoints; ++i)
        {
            if (t < mKnot[i])
            {
                break;
            }
        }
        --i;
    }

    return i;
}
//----------------------------------------------------------------------------

void BSplineBasis::Compute (Real t, unsigned int order, int& minIndex,int& maxIndex)
{
    assert(order <= 3);

    if (order >= 1)
    {
        if (!mBD1.size())
        {
            mBD1 = Allocate();
        }

        if (order >= 2)
        {
            if (!mBD2.size())
            {
                mBD2 = Allocate();
            }

            if (order >= 3)
            {
                if (!mBD3.size())
                {
                    mBD3 = Allocate();
                }
            }
        }
    }

    int i = GetKey(t);
    mBD0[0][i] = (Real)1;

    if (order >= 1)
    {
        mBD1[0][i] = (Real)0;
        if (order >= 2)
        {
            mBD2[0][i] = (Real)0;
            if (order >= 3)
            {
                mBD3[0][i] = (Real)0;
            }
        }
    }

    Real n0 = t - mKnot[i], n1 = mKnot[i+1] - t;
    Real invD0, invD1;
    int j;
    for (j = 1; j <= mDegree; j++)
    {
        invD0 = ((Real)1)/(mKnot[i+j] - mKnot[i]);
        invD1 = ((Real)1)/(mKnot[i+1] - mKnot[i-j+1]);

        mBD0[j][i] = n0*mBD0[j-1][i]*invD0;
        mBD0[j][i-j] = n1*mBD0[j-1][i-j+1]*invD1;

        if (order >= 1)
        {
            mBD1[j][i] = (n0*mBD1[j-1][i] + mBD0[j-1][i])*invD0;
            mBD1[j][i-j] = (n1*mBD1[j-1][i-j+1] - mBD0[j-1][i-j+1])*invD1;

            if (order >= 2)
            {
                mBD2[j][i] = (n0*mBD2[j-1][i] + ((Real)2)*mBD1[j-1][i])*invD0;
                mBD2[j][i-j] = (n1*mBD2[j-1][i-j+1] -
                    ((Real)2)*mBD1[j-1][i-j+1])*invD1;

                if (order >= 3)
                {
                    mBD3[j][i] = (n0*mBD3[j-1][i] +
                        ((Real)3)*mBD2[j-1][i])*invD0;
                    mBD3[j][i-j] = (n1*mBD3[j-1][i-j+1] -
                        ((Real)3)*mBD2[j-1][i-j+1])*invD1;
                }
            }
        }
    }

    for (j = 2; j <= mDegree; ++j)
    {
        for (int k = i-j+1; k < i; ++k)
        {
            n0 = t - mKnot[k];
            n1 = mKnot[k+j+1] - t;
            invD0 = ((Real)1)/(mKnot[k+j] - mKnot[k]);
            invD1 = ((Real)1)/(mKnot[k+j+1] - mKnot[k+1]);

            mBD0[j][k] = n0*mBD0[j-1][k]*invD0 + n1*mBD0[j-1][k+1]*invD1;

            if (order >= 1)
            {
                mBD1[j][k] = (n0*mBD1[j-1][k]+mBD0[j-1][k])*invD0 +
                    (n1*mBD1[j-1][k+1]-mBD0[j-1][k+1])*invD1;

                if (order >= 2)
                {
                    mBD2[j][k] = (n0*mBD2[j-1][k] +
                        ((Real)2)*mBD1[j-1][k])*invD0 +
                        (n1*mBD2[j-1][k+1] - ((Real)2)*mBD1[j-1][k+1])*invD1;

                    if (order >= 3)
                    {
                        mBD3[j][k] = (n0*mBD3[j-1][k] +
                            ((Real)3)*mBD2[j-1][k])*invD0 +
                            (n1*mBD3[j-1][k+1] - ((Real)3)*
                            mBD2[j-1][k+1])*invD1;
                    }
                }
            }
        }
    }

    minIndex = i - mDegree;
    maxIndex = i;
}
