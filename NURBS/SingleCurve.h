// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.

#pragma once

#include "NURBSGlobal.h"
#include "Curve.h"

namespace NURBS
{

template <typename Real>
class SingleCurve : public Curve<Real>
{
public:
    // Abstract base class.
    SingleCurve () {}
    SingleCurve (Real tmin, Real tmax);

    // Length-from-time and time-from-length.
    virtual Real GetLength (Real t0, Real t1);
    virtual Real GetTime (Real length, int iterations = 32, Real tolerance = (Real)1e-05);

    Curve<Real>::mTMin;
    Curve<Real>::mTMax;
    Curve<Real>::GetSpeed;
    Curve<Real>::GetTotalLength;

    static Real GetSpeedWithData (Real t, void* data);
};

typedef SingleCurve<float> SingleCurvef;
typedef SingleCurve<double> SingleCurved;

}
