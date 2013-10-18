#include "transform3d.h"

Eigen::MatrixXd transform_point3d(Eigen::MatrixXd& verts, Eigen::Matrix4d& trans)
{
	int NP  = verts.rows();
	Eigen::MatrixXd res(NP,4);
	res.col(0) = verts.col(0);
	res.col(1) = verts.col(1);
	res.col(2) = verts.col(2);
	res.col(3) = Eigen::VectorXd::Ones(NP,1);
	//std::cout << res << std::endl;

	//std::cout << trans * res.row(0).transpose() << std::endl;
	res = res * trans.transpose();
	//std::cout << res << std::endl;
	res.conservativeResize(NP, 3);
	 
	return res;
}
Eigen::Matrix4d create_translation3d(Eigen::Vector3d center)
{
	double dx(center.coeff(0)),dy(center.coeff(1)),dz(center.coeff(2));
	Eigen::Matrix4d trans;
	trans << 1,0,0,dx,
		    0,1,0,dy,
			0,0,1,dz,
			0,0,0,1;
	return trans;
}
Eigen::Matrix4d recenter_transform3d(Eigen::Matrix4d &transfo, Eigen::Vector3d& center)
{
	//% remove former translation part
	Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
	res.block(0, 0, 3, 3) = transfo.block(0, 0, 3, 3);

	//% create translations
	Eigen::Matrix4d t1 = create_translation3d(-center);
	Eigen::Matrix4d t2 = create_translation3d(center);

	//% compute translated transform
	res = t2*res*t1;
	return res;
}
Eigen::Matrix4d create_rotation3d_line_angle(Eigen::Vector3d& center,Eigen::Vector3d& v, double theta)
{
	//% normalize vector
	v.normalize();

	//% compute projection matrix P and anti-projection matrix
	Eigen::Matrix3d P = v * v.transpose();
	Eigen::Matrix3d Q;
	Q << 0, -v.coeff(2), v.coeff(1),
		v.coeff(2), 0, -v.coeff(0),
		-v.coeff(1), v.coeff(0), 0;
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

	//% compute vectorial part of the transform
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.block(0, 0, 3, 3) = P + (I - P)*cos(theta) + Q*sin(theta);

	//% add translation coefficient
	mat = recenter_transform3d(mat, center);
	
	return mat;
}

void vectorPts2MatrixPts(const std::vector<Eigen::Vector3d>& ptsin, Eigen::MatrixXd& ptsout)
{
	ptsout.resize(ptsin.size(), 3);
	for ( int i = 0; i < ptsin.size(); ++i)
	{
		ptsout.row(i) = ptsin[i];
	}
}
std::vector<Eigen::Vector3d> matrixPts2VectorPts(Eigen::MatrixXd& ptsin)
{
	std::vector<Eigen::Vector3d> ptsout;
	for ( int i = 0; i < ptsin.rows(); ++i)
	{
		ptsout.push_back( ptsin.row(i));
	}
	return ptsout;
}