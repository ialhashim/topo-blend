#pragma once
#include <Eigen/Dense>

#define ZERO_TOLERANCE 1e-8

typedef Eigen::Vector3d  Point_3;
typedef Eigen::Vector3d  Vector_3;

struct Segment_3{
	Segment_3(Point_3 p0 = Point_3(0,0,0), Point_3 p1 = Point_3(0,0,1)) { p[0] = p0; p[1] = p1; }
	Point_3 p[2];
	Point_3 point(int idx){ return p[idx]; }
	Vector_3 direction() { return (p[1] - p[0]).normalized(); }
};

struct Line_3{
	Line_3(Segment_3 s = Segment_3()) { point = s.point(0); direction = s.direction(); }
	Vector_3 point;
	Vector_3 direction;
};

struct Plane_3{
	Plane_3(Point_3 point = Vector_3(0,0,0), Vector_3 normal = Vector_3(0,0,1)) : p(point), n(normal.normalized()){
		double px = p.x(), py = p.y(), pz = p.z();
		double dx = n.x(), dy = n.y(), dz = n.z();
		pa = dx; pb = dy; pc = dz; pd = -dx*px - dy*py - dz*pz;
	}

	Point_3 p;
	Vector_3 n;
	double pa, pb, pc, pd;

	double a() const { return pa; }
	double b() const { return pb; }
	double c() const { return pc; }
	double d() const { return pd; }
};

static inline double squared_distance( Plane_3 plane, Point_3 point )
{
	double dist = plane.n.dot( Vector_3(point - plane.p) );
	return dist * dist; // squared
}


static inline double squared_distance_of_parallel( Line_3 mLine0, Line_3 mLine1 )
{
	Vector_3 diff = mLine0.point - mLine1.point;
	//double a01 = -mLine0.direction.dot(mLine1.direction);
	double b0 = diff.dot(mLine0.direction);
	double c = diff.squaredNorm();
	//double det = std::abs(1.0 - a01*a01);
	double s0, s1, sqrDist;

	// Lines are parallel, select any closest pair of points.
	s0 = -b0;
	s1 = (double)0;
	sqrDist = b0*s0 + c;

	// Account for numerical round-off errors.
	if (sqrDist < (double)0) sqrDist = (double)0;

	return sqrDist;
}

static inline double squared_distance( Line_3 mLine0, Line_3 mLine1 )
{
	Vector_3 diff = mLine0.point - mLine1.point;
	double a01 = -mLine0.direction.dot(mLine1.direction);
	double b0 = diff.dot(mLine0.direction);
	double c = diff.squaredNorm();
	double det = std::abs(1.0 - a01*a01);
	double b1, s0, s1, sqrDist;

	if (det >= ZERO_TOLERANCE)
	{
		// Lines are not parallel.
		b1 = -diff.dot(mLine1.direction);
		double invDet = ((double)1)/det;
		s0 = (a01*b1 - b0)*invDet;
		s1 = (a01*b0 - b1)*invDet;
		sqrDist = s0*(s0 + a01*s1 + ((double)2)*b0) +	s1*(a01*s0 + s1 + ((double)2)*b1) + c;
	}
	else
	{
		// Lines are parallel, select any closest pair of points.
		s0 = -b0;
		s1 = (double)0;
		sqrDist = b0*s0 + c;
	}

	// Account for numerical round-off errors.
	if (sqrDist < (double)0) sqrDist = (double)0;

	return sqrDist;
}

static inline Vector_3 cross_product( Vector_3 v1, Vector_3 v2 )
{
	return v1.cross(v2);
}

static inline Vector_3 cross_product( Segment_3 s1, Segment_3 s2 )
{
	return s1.direction().cross(s2.direction());
}
