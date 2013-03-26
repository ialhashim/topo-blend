#include "LineSegment.h"
using namespace NURBS;

Line::Line()
{
	length = 0;
	index = -1;
}

Line::Line(const Line& from)
{
	this->a = from.a;
	this->b = from.b;
	this->length = from.length;
	this->index = from.index;
}

Line::Line( const Vector3& from, const Vector3& to, int i)
{
	a = from;
	b = to;

	length = (a-b).norm();

	index = i;
}

Line::Line( const Vector3& start, const Vector3& direction, double length, int i)
{
	a = start;
	b = start + (direction.normalized() * length);

	length = length;

	index = i;
}

void Line::set( const Vector3& from, const Vector3& to )
{
	a = from;
	b = to;
	length = (a-b).norm();
	index = -1;
}

void Line::reverse()
{
    Vector3 temp = a;
	a = b;
	b = temp;
}

Vector3 Line::direction() const
{
	return b - a;
}

bool Line::hasPoint( const Vector3& point, double eps)
{
	double dist = (point - a).norm() + (point - b).norm();

	if(dist < length + eps)
		return true;
	else
		return false;
}

void Line::ClosestPoint(Point c, double &t, Point &d)
{
	Vector3 ab = b - a;
	// Project c onto ab, computing parameterized position d(t) = a + t*(b ?a)
	t = dot(c - a, ab) / dot(ab, ab);
	// If outside segment, clamp t (and therefore d) to the closest endpoint
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;
	// Compute projected position from the clamped t
	d = a + t * ab;
}

Vector3 Line::midPoint()
{
	return (a + b) * 0.5;
}

Vector3 Line::project( const Vector3& point )
{
	double time;
	Vec3d pos;
	ClosestPoint(point, time, pos);
	return pos;
}

Vector3 Line::pointAt( double time ) const
{
	double dist = time * length;
	return a + (direction().normalized() * dist);
}

double Line::timeAt( const Vector3& point )
{
	return dot((point - a), direction().normalized()) / length;
}

Pairdouble Line::lengthsAt( const Vector3& point )
{
	double dist1 = (point - a).norm();
	double dist2 = length - dist1;

	return Pairdouble(dist1, dist2);
}

Pairdouble Line::lengthsAt( double time )
{
    time = qMax(0.0, qMin(1.0, time)); // bounded
	return Pairdouble(length * time, length * (1.0 - time));
}

void Line::translateBy( const Vector3& delta )
{
	a += delta;
	b += delta;
}

void Line::intersectLine( const Line& S2, Vector3 & pa, Vector3 & pb, double Epsilon )
{
	double EPS = Epsilon; // experimental

    Vector3   u = this->b - this->a;
    Vector3   v = S2.b - S2.a;
    Vector3   w = this->a - S2.a;

	double    k = dot(u,u);			// (a) always >= 0
	double    j = dot(u,v);			// (b)
	double    c = dot(v,v);			// always >= 0
	double    d = dot(u,w);
	double    e = dot(v,w);
	double    D = k*c - j*j;       // always >= 0
	double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
	double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (D < EPS) { // the lines are almost parallel
		sN = 0.0;        // force using point P0 on segment S1
		sD = 1.0;        // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                // get the closest points on the infinite lines
		sN = (j*e - c*d);
		tN = (k*e - j*d);
		if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
			sN = sD;
			tN = e + j;
			tD = c;
		}
	}

	if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > k)
			sN = sD;
		else {
			sN = -d;
			sD = k;
		}
	}
	else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + j) < 0.0)
			sN = 0;
		else if ((-d + j) > k)
			sN = sD;
		else {
			sN = (-d + j);
			sD = k;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < EPS ? 0.0 : sN / sD);
	tc = (abs(tN) < EPS ? 0.0 : tN / tD);

	pa = this->pointAt(sc);
	pb = S2.pointAt(tc);
}

std::vector<Vector3> Line::uniformSample( int numSamples )
{
    std::vector<Vector3> result;

	double deltaLength = length / (numSamples - 1);
    Vector3 delta = deltaLength * direction().normalized();

	result.push_back(a);

	for(uint i = 1; i < (uint) numSamples; i++)
		result.push_back(result.back() + delta);

	return result;
}

Line::operator const std::vector<Vector3>()
{
	return uniformSample(2);
}
