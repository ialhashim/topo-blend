#include "geometry.h"
#include "transform3d.h"

double errorOfParallel(Eigen::Vector3d& d1, Eigen::Vector3d& d2)
{
    return 1 - sqrt( std::abs(d1.dot(d2))/(d1.norm()*d2.norm()) );
}
double errorOfOrthogonal(Eigen::Vector3d& d1, Eigen::Vector3d& d2)
{
    return sqrt( std::abs(d1.dot(d2))/(d1.norm()*d2.norm()) );
}
double errorOfCoplanar(Eigen::Vector3d &pt1, Eigen::Vector3d &normal1, Eigen::Vector3d &pt2, Eigen::Vector3d &normal2)
{
    double err1 = 1-std::abs(normal1.dot(normal2));

    Point_3 point(pt1.x(), pt1.y(), pt1.z());
    Plane_3 p2(Point_3(pt2.x(), pt2.y(), pt2.z()), Vector_3(normal2.x(), normal2.y(), normal2.z()));
    double dist1 = squared_distance(p2, point);
    return err1 + sqrt(dist1);
}

double errorOfLineInPlane(Segment_3 &l, Eigen::Vector3d& point, Eigen::Vector3d& normal)
{
	Plane_3 plane( Point_3(point.x(), point.y(), point.z()), 
				  Vector_3(normal.x(), normal.y(), normal.z()) );		
	double dist1 = squared_distance(plane, l.point(0));
	double dist2 = squared_distance(plane, l.point(1));
	return 0.5*(sqrt(dist1)+sqrt(dist2));
}


void distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2, double & min_dist, double &mean_dist, double &max_dist)
{
    min_dist = std::numeric_limits<double>::max(); mean_dist = 0.0; max_dist = 0.0; 

    for(int i = 0; i < (int) v1.rows(); ++i)
    {
        double minErr = std::numeric_limits<double>::max(), val(0.0);
        Eigen::Vector3d pt1 = v1.row(i);
        for ( int j = 0; j < (int) v2.rows(); ++j)
        {
            Eigen::Vector3d pt2 = v2.row(j);
            val = (pt1-pt2).norm();
            if ( val < minErr)
                minErr = val;
        }
        mean_dist += minErr;
        if(minErr > max_dist)
            max_dist = minErr;
		if(minErr < min_dist)
            min_dist = minErr;
    }
    mean_dist = mean_dist/v1.rows();
}

void reflect_points3d(const Eigen::MatrixXd& ptsin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout)
{
    ptsout.resize(ptsin.rows(), ptsin.cols() );
    for ( int i = 0; i < (int) ptsin.rows(); ++i)
    {
        Eigen::Vector3d ptout;
        reflect_point3d(ptsin.row(i), center, normal, ptout);
        ptsout.row(i) = ptout;
    }
}
void reflect_point3d(const Eigen::Vector3d& ptin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::Vector3d& ptout)
{
    Vector_3 v = normal * (center - ptin).dot(normal);
    ptout = ptin + v * 2;
}

void rotate_points3d(const Eigen::MatrixXd& ptsin, const Point_3& center, const Vector_3& direction, double angle, Eigen::MatrixXd& ptsout)
{
	Eigen::Vector3d c(center.x(), center.y(), center.z());
    Eigen::Vector3d dir(direction.x(), direction.y(), direction.z());
    Eigen::Matrix4d rotMat = create_rotation3d_line_angle(center, direction, angle);

	ptsout.resize(ptsin.rows(), ptsin.cols() );
    ptsout = transform_point3d(ptsin, rotMat);
}

Eigen::MatrixXd transform_point3d(const Eigen::MatrixXd& verts, const Eigen::Matrix4d& trans)
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
Eigen::Matrix4d create_translation3d(const Eigen::Vector3d center)
{
	double dx(center.coeff(0)),dy(center.coeff(1)),dz(center.coeff(2));
	Eigen::Matrix4d trans;
	trans << 1,0,0,dx,
		    0,1,0,dy,
			0,0,1,dz,
			0,0,0,1;
	return trans;
}
Eigen::Matrix4d recenter_transform3d(const Eigen::Matrix4d &transfo, const Eigen::Vector3d& center)
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
Eigen::Matrix4d create_rotation3d_line_angle(const Eigen::Vector3d& center, Eigen::Vector3d v, double theta)
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

