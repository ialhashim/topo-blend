// Adapted from WildMagic
#pragma once

#include "Curve.h"

namespace NURBS
{

class SingleCurve : public Curve{
public:
    // Abstract base class.
    SingleCurve () {}
    SingleCurve (Real tmin, Real tmax);

    // Length-from-time and time-from-length.
    virtual Real GetLength (Real t0, Real t1) ;
    virtual Real GetTime (Real length, int iterations = 32, Real tolerance = (Real)TimeTolerance) ;

protected:
    using Curve::mTMin;
    using Curve::mTMax;
    using Curve::GetSpeed;
    using Curve::GetTotalLength;

    static Real GetSpeedWithData (Real t, void* data);
};

}
