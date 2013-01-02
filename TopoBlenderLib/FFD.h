#pragma once

#include "SurfaceMeshModel.h"

struct ControlPoint : public Vec3d{
	ControlPoint( Vec3d pos, int useIndex, Vec3i gridIdx ){ 
		set(pos);
		index = useIndex; grid = gridIdx; 
	}
	void set(const Vec3d & newPos){ Vec3d & myPos = *this; myPos = newPos; } 
	int index;
	Vec3i grid;
};

enum FFD_FitType {BoundingBoxFFD, VolumeFFD};

class FFD
{
public:
	FFD( const std::vector<Vec3d> & pointSoup, FFD_FitType fit_type = BoundingBoxFFD, Vec3i res = Vec3i(2) );
    FFD( Surface_mesh * src_mesh = NULL, FFD_FitType fit_type = BoundingBoxFFD, Vec3i res = Vec3i(2) );
    void init( Surface_mesh*, FFD_FitType, Vec3i );

	Surface_mesh * mesh;
	double width, length, height;
	Vec3d center;
	Vec3i resolution;
    Surface_mesh::Vertex_property<Vec3d> mesh_points;
	Surface_mesh temp_mesh;

    std::vector< ControlPoint* > control_points;
    std::vector< std::vector< std::vector < int > > > pointsGridIdx;

	// Every time you modify set of control points you just call this to update
	void apply();

private:
	Vec3d deformVertexLocal( const Vec3d & localPoint );
	Vec3d getWorldCoordinate(const Vec3d & pLocal);
	Vec3d getLocalCoordinates( const Vec3d & p );
	Vec3d mP, mS, mT, mU;	// the local frame coordinates
    std::vector<Vec3d> meshVerticesLocal;

    // A) Bounding Box fitting
    void bbFit(Vec3i res);

public:
    // B) Manually setup FFD
    void customVolume( Vec3i res, Vec3d location, double spacing, std::map<int,Vec3d> pnts );
    std::map<int,Vec3d> applyCustom();
    std::map<int,Vec3d> fixedPointsLocal;

	// Quick access
	inline Vec3d outputPoint( int idx );
	std::vector<Vec3d> outputPoints();
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
