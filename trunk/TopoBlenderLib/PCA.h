#pragma once

#include "SurfaceMeshModel.h"

#include <Eigen/Core>
#include <Eigen/Eigen>
using namespace Eigen;

struct PCA{
    static Vec3d mainAxis( const std::vector<Vec3d> & points)
    {
        Vec3d first = Vec3d(0);
        Vec3d second = Vec3d(0);
        Vec3d third = Vec3d(0) ;

        return mainAxis(points, first, second, third);
    }

    static Vec3d mainAxis( const std::vector<Vec3d> & points, Vec3d & first, Vec3d & second, Vec3d & third )
	{
		int n = points.size();

		double	x, y, z, xx, yy, zz, xy, xz, yz,
			sumX (0.0), sumY (0.0), sumZ (0.0),
			sumX2(0.0), sumY2(0.0), sumZ2(0.0),
			sumXY(0.0), sumXZ(0.0), sumYZ(0.0);

		Vec3d mean = Vec3d(0);

        foreach(SurfaceMesh::Point p, points){
			mean += p;
			x = p.x();	y = p.y();	z = p.z();

			sumX  += x / n		;	sumY  += y / n		;	sumZ  += z / n;
			sumX2 += x * x / n	;	sumY2 += y * y / n	;	sumZ2 += z * z / n;
			sumXY += x * y / n	;	sumXZ += x * z / n	;	sumYZ += y * z / n;
		}

		mean /= points.size();

		xx = sumX2 - sumX * sumX; yy = sumY2 - sumY * sumY;	zz = sumZ2 - sumZ * sumZ;
		xy = sumXY - sumX * sumY; xz = sumXZ - sumX * sumZ;	yz = sumYZ - sumY * sumZ;

		Matrix3d covariance;
		covariance <<	xx, xy, xz,
						xy, yy, yz,
						xz, yz, zz;

		SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);
		Vector3d e_values = es.eigenvalues();
		Matrix3d e_vectors = es.eigenvectors();

		Vector3d a = e_vectors.col(0);
		Vector3d b = e_vectors.col(1);
		Vector3d c = e_vectors.col(2);

		first = Vec3d(a[0], a[1], a[2]);
		second = Vec3d(b[0], b[1], b[2]);
		third = Vec3d(c[0], c[1], c[2]);

		return third;
	}
};
