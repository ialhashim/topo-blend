#include "geometry.h"
#include "transform3d.h"

int extractCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, int pointsLevel)
{
    if ( pointsLevel == 2)
    {
    	SurfaceMesh::Model * m1 = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
    	SurfaceMesh::Vector3VertexProperty pts1 = m1->vertex_coordinates();
    	double* tmp;
    	foreach(Vertex v1, m1->vertices())
    	{
    		tmp = pts1[v1].data();
    		mcpts.push_back( Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) );
    	}
    }
	else
	{
		if ( Structure::CURVE == n->type() )
		{
			Structure::Curve* c = dynamic_cast<Structure::Curve *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) c->numCtrlPnts(); ++i)
				{
					mcpts.push_back( c->controlPoint(i));
				}
			}
			else
			{
				mcpts.push_back( c->controlPoint(0) );
				mcpts.push_back( c->controlPoint( c->numCtrlPnts()-1 ) );
			}
		}
		else
		{
			Structure::Sheet* s = dynamic_cast<Structure::Sheet *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) s->numCtrlPnts(); ++i)
				{
					mcpts.push_back( s->controlPoint(i));
				}
			}
			else
			{
				int nu = s->numUCtrlPnts(), nv = s->numVCtrlPnts();
				mcpts.push_back( s->surface.GetControlPoint(0,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,nv-1) );
				mcpts.push_back( s->surface.GetControlPoint(0,nv-1) );
			}
		}
	}

	return mcpts.size();
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
void reflectPoints(const Eigen::MatrixXd& ptsin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout)
{
    ptsout.resize(ptsin.rows(), ptsin.cols() );
    for ( int i = 0; i < (int) ptsin.rows(); ++i)
    {
        Eigen::Vector3d ptout;
        reflectPoint(ptsin.row(i), center, normal, ptout);
        ptsout.row(i) = ptout;
    }
}
void reflectPoint(const Eigen::Vector3d& ptin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::Vector3d& ptout)
{
    Vector_3 v = normal * (center - ptin).dot(normal);
    ptout = ptin + v * 2;
}

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
    for ( int i = 0; i < (int)ptsin.size(); ++i)
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
