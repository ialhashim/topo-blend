#pragma once

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

#include "SurfaceMeshHelper.h"

using namespace SurfaceMeshTypes;

static inline Scalar deg_to_rad(const Scalar& _angle)
{ return M_PI*(_angle/180.0); }

static inline Scalar rad_to_deg(const Scalar& _angle)
{ return 180.0*(_angle/M_PI); }

static inline Scalar sane_aarg(Scalar _aarg){
	if (_aarg < -1)
		_aarg = -1;
	else if (_aarg >  1)
		_aarg = 1;
	return _aarg;
}

static inline Scalar angle(Scalar _cos_angle, Scalar _sin_angle)
{//sanity checks - otherwise acos will return NAN
	_cos_angle = sane_aarg(_cos_angle);
	return (Scalar) _sin_angle >= 0 ? acos(_cos_angle) : -acos(_cos_angle);
}

// Helper function
static Scalar calc_dihedral_angle(SurfaceMeshModel * mesh, SurfaceMeshModel::Halfedge _heh)
{
	if (mesh->is_boundary(mesh->edge(_heh)))
	{//the dihedral angle at a boundary edge is 0
		return 0;
	}

	Vector3FaceProperty normal = mesh->face_property<Vector3>( FNORMAL );
	Vector3VertexProperty points = mesh->vertex_property<Vector3>( VPOINT );

	const Normal& n0 = normal[mesh->face(_heh)];
	const Normal& n1 = normal[mesh->face(mesh->opposite_halfedge(_heh))];

	Normal he = points[mesh->to_vertex(_heh)] - points[mesh->from_vertex(_heh)];

	Scalar da_cos = dot(n0, n1);

	//should be normalized, but we need only the sign
	Scalar da_sin_sign = dot(cross(n0, n1), he);
	return angle(da_cos, da_sin_sign);
}
