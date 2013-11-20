#pragma once
#include <Eigen/Dense>
#include <vector>
#include "geometry.h"

//////////////////////////////////////////
// compute min, mean & max distance between 2 sets of points
void distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2, double & min_dist, double &mean_dist, double &max_dist);


//////////////////////////////////////////
void reflect_points3d(const Eigen::MatrixXd& ptsin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout);
void reflect_point3d(const Eigen::Vector3d& ptin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::Vector3d& ptout);

Eigen::MatrixXd transform_point3d(Eigen::MatrixXd& verts, Eigen::Matrix4d& trans);
Eigen::Matrix4d create_translation3d(Eigen::Vector3d center);
Eigen::Matrix4d create_rotation3d_line_angle(Eigen::Vector3d& center,Eigen::Vector3d& v, double theta);


//////////////////////////////////////////
double errorOfParallel(Eigen::Vector3d& d1, Eigen::Vector3d& d2);
double errorOfOrthogonal(Eigen::Vector3d& d1, Eigen::Vector3d& d2);
double errorOfCoplanar(Eigen::Vector3d &pt1, Eigen::Vector3d &normal1, Eigen::Vector3d &pt2, Eigen::Vector3d &normal2);
double errorOfLineInPlane(Segment_3 &l, Eigen::Vector3d& point, Eigen::Vector3d& normal);