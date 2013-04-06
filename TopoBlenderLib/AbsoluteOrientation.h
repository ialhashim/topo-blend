#pragma once

#include "SurfaceMeshHelper.h"

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#define V2E(vec) ((Eigen::Vector3d(vec[0], vec[1], vec[2])))
#define E2V(vec) ((Vec3d(vec[0], vec[1], vec[2])))

struct AbsoluteOrientation{

	static void compute( Vec3d Xa,Vec3d Ya,Vec3d Za, Vec3d Xb,Vec3d Yb,Vec3d Zb, Eigen::Quaterniond &result );
	static void compute( std::vector<Vec3d> &left, std::vector<Vec3d> &right, Eigen::Quaterniond &result );

	// Experiments
	static void minOnT( std::vector<Vec3d> &frameA, std::vector<Vec3d> &frameB, Eigen::Quaterniond &result );
};
