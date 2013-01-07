#pragma once

#include "ParametricSurface.h"
#include "BSplineBasis.h"

// For visualization
struct SurfaceQuad{	Vector3 p[4]; Normal n[4]; };

class NURBSRectangle : public ParametricSurface
{
public:

    NURBSRectangle(){}

    // construction and destruction.   The caller is responsible for deleting
    // the input arrays if they were dynamically allocated.  Internal copies
    // of the arrays are made, so to dynamically change control points,
    // control weights, or knots, you must use the 'SetControlPoint',
    // 'GetControlPoint', 'SetControlWeight', 'GetControlWeight', and 'Knot'
    // member functions.

    // The homogeneous input points are (x,y,z,w) where the (x,y,z) values are
    // stored in the akCtrlPoint array and the w values are stored in the
    // afCtrlWeight array.  The output points from curve evaluations are of
    // the form (x',y',z') = (x/w,y/w,z/w).

    // Spline types for curves are
    //   open uniform (OU)
    //   periodic uniform (PU)
    //   open nonuniform (ON)
    // For tensor product surfaces, you have to choose a type for each of two
    // dimensions, leading to nine possible spline types for surfaces.  The
    // constructors below represent these choices.

    NURBSRectangle(Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight,
                   int uDegree, int vDegree, bool uLoop, bool vLoop, bool uOpen, bool vOpen);

    static NURBSRectangle createSheet(Scalar width = 2.0, Scalar length = 1.0, Vector3 center = Vector3(0),
                                      Vector3 dU = Vector3(1,0,0), Vector3 dV = Vector3(0,0,1));

    int GetNumCtrlPoints (int dim) ;
    int GetDegree (int dim) ;
    bool IsOpen (int dim) ;
    bool IsUniform (int dim) ;
    bool IsLoop (int dim) ;

    // Control points and weights may be changed at any time.  If either input
    // index is invalid, GetControlPoint returns a vector whose components
    // are all MAX_REAL, and GetControlWeight returns MAX_REAL.
    void SetControlPoint (int uIndex, int vIndex,  Vector3& ctrl);
    Vector3 GetControlPoint (int uIndex, int vIndex) ;
    void SetControlWeight (int uIndex, int vIndex, Real weight);
    Real GetControlWeight (int uIndex, int vIndex) ;

	std::vector<Vector3> GetControlPointsU (int vIndex = 0);
	std::vector<Vector3> GetControlPointsV (int uIndex = 0);

    // The knot values can be changed only if the surface is nonuniform in the
    // selected dimension and only if the input index is valid.  If these
    // conditions are not satisfied, GetKnot returns MAX_REAL.
    void SetKnot (int dim, int i, Real knot);
    Real GetKnot (int dim, int i) ;

    // The spline is defined for 0 <= u <= 1 and 0 <= v <= 1.  The input
    // values should be in this domain.  Any inputs smaller than 0 are clamped
    // to 0.  Any inputs larger than 1 are clamped to 1.
    virtual Vector3 P (Real u, Real v) ;
    virtual Vector3 PU (Real u, Real v) ;
    virtual Vector3 PV (Real u, Real v) ;
    virtual Vector3 PUU (Real u, Real v) ;
    virtual Vector3 PUV (Real u, Real v) ;
    virtual Vector3 PVV (Real u, Real v) ;

    // If you need position and derivatives at the same time, it is more
    // efficient to call these functions.  Pass the addresses of those
    // quantities whose values you want.  You may pass 0 in any argument
    // whose value you do not want.
    void Get (Real u, Real v, Vector3& pos = Vector3(0), Vector3& derU= Vector3(0),
        Vector3& derV= Vector3(0), Vector3& derUU= Vector3(0), Vector3& derUV= Vector3(0), Vector3& derVV= Vector3(0));

	// General properties
	Real aspectU();
	Real aspectV();

	Real avgCtrlEdgeLength();

	// Cached visualization
	std::vector<SurfaceQuad> quads;
    void generateSurfaceQuads( int resolution );

	// Discretization
	void uniformCoordinates(std::vector<Real> & valU, std::vector<Real> & valV, int resolution = 10, int u = 0, int v = 0);
	std::vector< std::vector<Vector3> > generateSurfaceTris( Real resolution );
	void generateSurfacePoints( Scalar stepSize, std::vector< std::vector<Vector3> > & points, 
		std::vector<Real> & valU = std::vector<Real>(), std::vector<Real> & valV = std::vector<Real>());
	void generateSurfacePointsCoords( Scalar stepSize, std::vector< std::vector<Vec4d> > & points);

	// Control cage / network
	std::vector< std::vector<Vector3> > triangulateControlCage();
	Vector3 projectOnControl( Real u, Real v );

	// Projection
	Vec4d timeAt(const Vector3 & pos);
	Vec4d timeAt( const Vector3 & pos, Vec4d & bestUV, Vec4d & minRange, Vec4d & maxRange, Real currentDist, Real threshold = 1e-4 );
	std::vector<Vec4d> NURBSRectangle::timeAt( const std::vector<Vector3> & pos, Real threshold );

	// Modifiers
	void bend( Scalar amount, int bendDirection = 0 );
	void translate( const Vec3d & delta );
	void scale(Scalar scaleFactor);

	// Intersection
	std::vector<Vec3d> NURBSRectangle::intersect( NURBSRectangle & other, double resolution, 
		std::vector<Vec4d> & coordMe = std::vector<Vec4d>(), std::vector<Vec4d> & coordOther = std::vector<Vec4d>() );

protected:
    // Replicate the necessary number of control points when the Create
    // function has bLoop equal to true, in which case the spline surface
    // must be a closed surface in the corresponding dimension.
    void CreateControl (Array2D_Vector3 ctrlPoint, Array2D_Real ctrlWeight);

public:
    int mNumUCtrlPoints, mNumVCtrlPoints;
    Array2D_Vector3 mCtrlPoint;  //   ctrl[unum][vnum]
    Array2D_Real mCtrlWeight;    // weight[unum][vnum]
    bool mLoop[2];
    BSplineBasis mBasis[2];
    int mUReplicate, mVReplicate;

	// DEBUG:
	std::vector<Vector3> debugPoints;
};

