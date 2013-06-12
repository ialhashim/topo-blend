#pragma once

#include "Curve.h"

namespace NURBS{

typedef std::pair<double, double> Pairdouble;

class Line
{
public:
    Eigen::Matrix<double,3,1,Eigen::DontAlign> a, b;
	double length;
	int index;

	Line();
	Line(const Line&);
    Line(const Vector3& from, const Vector3& to, int i = -1);
    Line(const Vector3& start, const Vector3& direction, double length, int i = -1);

    void set(const Vector3& from, const Vector3& to);

    Vector3 direction() const;
    bool hasPoint(const Vector3& point, double eps = 1e-10);
	void reverse();
	
    Vector3 midPoint();
    Vector3 pointAt(double time) const;
    Vector3 project(const Vector3& point);
    double timeAt(const Vector3& point);
    Pairdouble lengthsAt(const Vector3& point);
	Pairdouble lengthsAt(double time);

    std::vector< Eigen::Matrix<double,3,1,Eigen::DontAlign> > uniformSample(int numSamples);
    operator const std::vector< Eigen::Matrix<double,3,1,Eigen::DontAlign> >();

    void translateBy(const Vector3& delta);
    double distanceToUnbounded(const Vector3& point);
	void ClosestPoint(Point c, double &t, Point &d);
    void intersectLine( const Line& S2, Vector3 & pa, Vector3 & pb, double Epsilon = 1e-10 );
};

}
