#pragma once
#include <Eigen/Dense>
#include <vector>

#include "StructureGraph.h"

int extractCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, int pointsLevel);

void vectorPts2MatrixPts(const std::vector<Eigen::Vector3d>& ptsin, Eigen::MatrixXd& ptsout);
std::vector<Eigen::Vector3d> matrixPts2VectorPts(Eigen::MatrixXd& ptsin);

double distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2);
void distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2, double &mean_dist, double &max_dist);

void reflectPoints(const Eigen::MatrixXd& ptsin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout);
void reflectPoint(const Eigen::Vector3d& ptin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::Vector3d& ptout);

Eigen::MatrixXd transform_point3d(Eigen::MatrixXd& verts, Eigen::Matrix4d& trans);
Eigen::Matrix4d create_translation3d(Eigen::Vector3d center);
Eigen::Matrix4d create_rotation3d_line_angle(Eigen::Vector3d& center,Eigen::Vector3d& v, double theta);

