#pragma once

#include "SurfaceMeshHelper.h"

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct AbsoluteOrientation{

	static void compute( Eigen::Vector3d Xa,Eigen::Vector3d Ya,Eigen::Vector3d Za, Eigen::Vector3d Xb,Eigen::Vector3d Yb,Eigen::Vector3d Zb, Eigen::Quaterniond &result );
	static void compute( std::vector<Eigen::Vector3d> &left, std::vector<Eigen::Vector3d> &right, Eigen::Quaterniond &result );

	// Experiments
	static void minOnT( std::vector<Eigen::Vector3d> &frameA, std::vector<Eigen::Vector3d> &frameB, Eigen::Quaterniond &result );
};
