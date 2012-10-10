// Adapted from WildMagic
#pragma once

#include "Curve.h"

template <typename Real>
class SingleCurve : public Curve<Real>
{
public:
    // Abstract base class.
    SingleCurve () {}
    SingleCurve (Real tmin, Real tmax);

    // Length-from-time and time-from-length.
    virtual Real GetLength (Real t0, Real t1) const;
    virtual Real GetTime (Real length, int iterations = 32, Real tolerance = (Real)1e-06) const;

protected:
    using Curve<Real>::mTMin;
    using Curve<Real>::mTMax;
    using Curve<Real>::GetSpeed;
    using Curve<Real>::GetTotalLength;

    static Real GetSpeedWithData (Real t, void* data);
};

typedef SingleCurve<float> SingleCurve3f;
typedef SingleCurve<double> SingleCurve3d;
