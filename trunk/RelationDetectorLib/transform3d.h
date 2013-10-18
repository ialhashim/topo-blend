//jjcao begin
#pragma once
#include <Eigen/Dense>
#include <vector>

Eigen::MatrixXd transform_point3d(Eigen::MatrixXd& verts, Eigen::Matrix4d& trans);
Eigen::Matrix4d create_translation3d(Eigen::Vector3d center);
Eigen::Matrix4d create_rotation3d_line_angle(Eigen::Vector3d& center,Eigen::Vector3d& v, double theta);

void vectorPts2MatrixPts(const std::vector<Eigen::Vector3d>& ptsin, Eigen::MatrixXd& ptsout);
std::vector<Eigen::Vector3d> matrixPts2VectorPts(Eigen::MatrixXd& ptsin);

//jjcao end