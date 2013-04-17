#pragma once
// Based on "Computation of Rotation Minimizing Frames" Wang et al. 2008

#include <vector>
typedef unsigned int uint;

#define ZERO_NORM 1e-10

#include "qglviewer/quaternion.h"
using namespace qglviewer;

class RMF{
public:
	RMF(){}

    RMF(const std::vector<Vec3d> & fromPoints, bool isCompute = true)
	{
		if(fromPoints.size() == 0) return;

		point = fromPoints;

		if(fromPoints.size() > 1 && (fromPoints[0] - fromPoints[1]).norm() > ZERO_NORM)
            if(isCompute) compute();
		else
		{
			U.push_back(Frame::fromRS(Vec3d(1,0,0), Vec3d(0,1,0)));
			U.back().center = fromPoints.front();
		}
	}

	void compute()
	{
		Vec3d firstT = (point[1] - point[0]).normalized();
		Vec3d firstR = orthogonalVector( firstT );

		compute( firstR );
	}

	void compute( Vec3d firstR )
	{
		// Reset computation
		std::vector<Vec3d> tangent;

		// Estimate tangents
		for(uint i = 0; i < point.size() - 1; i++){
			Vec3d t = point[i+1] - point[i];
			if(i > 0 && t.norm() < ZERO_NORM) 
				tangent.push_back(tangent.back());
			else
				tangent.push_back((t).normalized());
		}
		tangent.push_back(tangent.back());

		Vec3d firstT = tangent.front().normalized();

		// Make sure firstR is perpendicular to plane of first tangent
		firstR = pointOnPlane(firstR, firstT);
		if(firstR.norm() > ZERO_NORM) 
			firstR.normalize();
		else 
			firstR = orthogonalVector(firstT);

		// First frame
		Frame firstFrame = Frame::fromTR( firstT, firstR.normalized() );

		U.clear();
		U.push_back( firstFrame );

		// Double reflection method: compute rotation minimizing frames
		for(uint i = 0; i < point.size() - 1; i++)
		{
			Vec3d ri = U.back().r, ti = U.back().t, tj = tangent[i+1];

			/*1 */ Vec3d v1 = point[i+1] - point[i]; if(v1.norm() < ZERO_NORM){ U.push_back(U.back()); continue; }
			/*2 */ double c1 = dot(v1,v1);
			/*3 */ Vec3d rLi = ri - (2.0 / c1) * dot(v1, ri) * v1;
			/*4 */ Vec3d tLi = ti - (2.0 / c1) * dot(v1, ti) * v1;
			/*5 */ Vec3d v2 = tj - tLi;
			/*6 */ double c2 = dot(v2,v2);
			/*7 */ Vec3d rj = rLi - (2.0 / c2) * dot(v2, rLi) * v2;
			/*8 */ Vec3d sj = cross(tj,rj);

			U.push_back(Frame::fromST(sj.normalized(), tj.normalized()));
		}

		// RMF Visualization
		for(int i = 0; i < (int)point.size(); i++)
			U[i].center = point[i];
	}

	void generate()
	{
		std::vector<Vec3d> tangent;

		// Estimate tangents
		for(uint i = 0; i < point.size() - 1; i++){
			Vec3d t = point[i+1] - point[i];
			if(i > 0 && t.norm() < ZERO_NORM) 
				tangent.push_back(tangent.back());
			else
				tangent.push_back((t).normalized());
		}
		tangent.push_back(tangent.back());

		// generate frames
		Vec X(1, 0, 0), Z(0, 0, 1);
		for(uint i = 0; i < point.size(); i++){
			Vec3d t = tangent[i];

			qglviewer::Quaternion q(Z, Vec(t));
			Vec r = q * X;

			U.push_back(Frame::fromTR(t, Vec3d(r.x, r.y, r.z)));
		}
	}

	inline uint count() { return point.size(); }

	class Frame{ 
	public:
		Vec3d r, s, t;
		Vec3d center; // optional

		Frame(){ r = Vec3d(1,0,0); s = Vec3d(0,1,0); t = Vec3d(0,0,1); }
		Frame(const Vec3d& R, const Vec3d& S, const Vec3d& T) { r = R; s = S; t = T; normalize(); }

		static Frame fromTR(const Vec3d& T, const Vec3d& R) { return Frame(R, cross(T,R), T); }
		static Frame fromRS(const Vec3d& R, const Vec3d& S) { return Frame(R, S, cross(R,S)); }
		static Frame fromST(const Vec3d& S, const Vec3d& T) { return Frame(cross(S,T), S, T); }
		static Frame fromT(const Vec3d& T) { Vec3d R = orthogonalVector(T.normalized()).normalized(); return fromTR(T,R); }

		static Vec3d orthogonalVector(const Vec3d& n) {
			if ((abs(n.y()) >= 0.9 * abs(n.x())) && abs(n.z()) >= 0.9 * abs(n.x())) return Vec3d(0.0, -n.z(), n.y());
			else if ( abs(n.x()) >= 0.9 * abs(n.y()) && abs(n.z()) >= 0.9 * abs(n.y()) ) return Vec3d(-n.z(), 0.0, n.x());
			else return Vec3d(-n.y(), n.x(), 0.0);
		}

		void normalize() { r.normalize(); s.normalize(); t.normalize(); } ;
	};

	static inline Vector3 pointOnPlane(Vector3 p, Vector3 plane_normal, Scalar plane_d = 0)
	{
		Scalar t = dot(plane_normal, p) - plane_d;
		return p - (t * plane_normal);
	}

	inline Frame frameAt(double t){
		return U[ (qRanged(0.0, t, 1.0) * (U.size() - 1)) ];
	}

	std::vector<Vec3d> point;
	std::vector<Frame> U;
};
