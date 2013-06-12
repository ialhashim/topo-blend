#pragma once

#include "SurfaceMeshModel.h"
using namespace Eigen;

struct ControlPoint : public Vector3d{
	ControlPoint( Vector3d pos, int useIndex, Vector3i gridIdx ){ 
		set(pos);
		index = useIndex; grid = gridIdx; 
	}
	void set(const Vector3d & newPos){ Vector3d & myPos = *this; myPos = newPos; } 
	int index;
	Vector3i grid;
};

enum FFD_FitType {BoundingBoxFFD, VolumeFFD};

class FFD
{
public:
	FFD( const std::vector<Vector3d> & pointSoup, FFD_FitType fit_type = BoundingBoxFFD, Vector3i res = Vector3i(2) );
    FFD( Surface_mesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD, Vector3i res = Vector3i(2) );
    void init( Surface_mesh*, FFD_FitType, Vector3i );

	Surface_mesh * mesh;
	double width, length, height;
	Eigen::Vector3d center;
	Vector3i resolution;
    Surface_mesh::Vertex_property<Vector3d> mesh_points;
	Surface_mesh temp_mesh;

    std::vector< ControlPoint* > control_points;
    std::vector< std::vector< std::vector < int > > > pointsGridIdx;

	// Every time you modify set of control points you just call this to update
	void apply();

private:
	Vector3d deformVertexLocal( const Vector3d & localPoint );
	Vector3d getWorldCoordinate(const Vector3d & pLocal);
	Vector3d getLocalCoordinates( const Vector3d & p );
	Vector3d mP, mS, mT, mU;	// the local frame coordinates
    std::vector<Vector3d> meshVerticesLocal;

    // A) Bounding Box fitting
    void bbFit(Vector3i res);

public:
    // B) Manually setup FFD
    void customVolume( Vector3i res, Vector3d location, double spacing, std::map<int,Vector3d> pnts );
    std::map<int,Vector3d> applyCustom();
    std::map<int,Vector3d> fixedPointsLocal;

	// Quick access
	inline Vector3d outputPoint( int idx );
	std::vector<Vector3d> outputPoints();
};

// math helpers
inline int fact(int x){
	if ( x == 0 ) return 1;
	int f = 1;
	for (int i=1;i<=x;++i) f*=i;
	return f;
}
inline int Coef(int a, int b){
	// return the aCb (binomial coefficients)
	assert(a>=b);
	return fact(a) / (fact(b) * fact(a-b));
}

Q_DECLARE_METATYPE(FFD*)
