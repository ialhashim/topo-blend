// Adapted from WildMagic under Boost Software License - Version 1.0
#pragma once

#include "SurfaceMeshModel.h"
#include "SurfaceMeshHelper.h"
using namespace SurfaceMeshTypes;

#define MAX_REAL std::numeric_limits<SurfaceMeshTypes::Scalar>::max()
#define REAL_ZERO_TOLERANCE 1e-08

typedef std::vector< std::vector<Vector3> > Array2D_Vector3;
typedef std::vector< Scalar > Array1D_Real;
typedef std::vector< Array1D_Real > Array2D_Real;

template <typename Real>
class Curve
{

public:
    // Abstract base class.
    Curve () {}
    Curve (Real tmin, Real tmax);
    virtual ~Curve ();

    // Interval on which curve parameter is defined.  If you are interested
    // in only a subinterval of the actual domain of the curve, you may set
    // that subinterval with SetTimeInterval.  This function requires that
    // tmin < tmax.
    Real GetMinTime () const;
    Real GetMaxTime () const;
    void SetTimeInterval (Real tmin, Real tmax);

    // Position and derivatives.
    virtual Vector3 GetPosition (Real t) const = 0;
    virtual Vector3 GetFirstDerivative (Real t) const = 0;
    virtual Vector3 GetSecondDerivative (Real t) const = 0;
    virtual Vector3 GetThirdDerivative (Real t) const = 0;

    // Differential geometric quantities.
    Real GetSpeed (Real t) const;
    virtual Real GetLength (Real t0, Real t1) const = 0;
    Real GetTotalLength () const;
    Vector3 GetTangent (Real t) const;
    Vector3 GetNormal (Real t) const;
    Vector3 GetBinormal (Real t) const;
    void GetFrame (Real t, Vector3& position, Vector3& tangent,
        Vector3& normal, Vector3& binormal) const;
    Real GetCurvature (Real t) const;
    Real GetTorsion (Real t) const;

    // Inverse mapping of s = Length(t) given by t = Length^{-1}(s).
    virtual Real GetTime (Real length, int iterations = 32, Real tolerance = (Real)1e-06) const = 0;

    // Subdivision.
    void SubdivideByTime (int numPoints, Vector3*& points) const;
    void SubdivideByLength (int numPoints, Vector3*& points) const;

protected:
    // Curve parameter is t where tmin <= t <= tmax.
    Real mTMin, mTMax;
};

typedef Curve<float> Curve3f;
typedef Curve<double> Curve3d;
