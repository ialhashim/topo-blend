#pragma once

#include "SurfaceMeshHelper.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
using namespace Eigen;

#define (vec) (Eigen::Vector3d(vec[0], vec[1], vec[2]))
#define (vec) (Vec3d(vec[0], vec[1], vec[2]))

class PCA3
{
public:
    PCA3(const std::vector<Vec3d> & points)
	{
		int n = points.size();

        double	x, y, z, xx, yy, zz, xy, xz, yz,
			sumX (0.0), sumY (0.0), sumZ (0.0),
			sumX2(0.0), sumY2(0.0), sumZ2(0.0),
			sumXY(0.0), sumXZ(0.0), sumYZ(0.0);

        mean = Vec3d(0);

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
		covariance << xx, xy, xz,
					  xy, yy, yz,
					  xz, yz, zz;

		// Solve for eigenvalues and eigenvectors.
		// Eigen values are sorted in ascending order,
		SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);
		e_values = es.eigenvalues();
		e_vectors = es.eigenvectors();
	}

	Vec3d eigenvalues()
	{
		return (e_values);
	}

	std::vector<Vec3d> eigenvectors()
	{
		std::vector<Vec3d> vectors;
		for (int i = 2;i >= 0;i--)
		{
			// Descending order
			vectors.push_back((e_vectors.col(i)));
		}

		return vectors;
	}

    static void setPlane(const std::vector<Vec3d> & points, Vec3d & center, Vec3d & normal)
    {
        PCA3 pca(points);
        center = pca.mean;
        Vector3d n = pca.e_vectors.col(1).normalized().cross(pca.e_vectors.col(2).normalized());
        normal = Vec3d(n[0], n[1], n[2]);
    }

	Vec3d center()
	{
		return mean;
	}

private:
	Vector3d e_values;
	Matrix3d e_vectors;
    Vec3d mean;
};
