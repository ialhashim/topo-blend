#pragma once
#include "StructureGraph.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
using namespace Eigen;

static QString NODE_CENTER = "center2D";

struct GraphEmbed
{
    static void embed( Structure::Graph * graph )
    {
        // Get nodes centers
        QMap<int, Node*> nmap;
        std::vector<Vector3> centers;
        foreach(Node * n, graph->nodes){
            nmap[nmap.size()] = n;
            centers.push_back( n->bbox().center() );
        }
        int N = centers.size();

        // PCA
        Vector3 plane_center(0), plane_normal(0); Scalar plane_d = 0;
        planePCA(centers, plane_center, plane_normal, plane_d);

        // Project to PCA plane, to XY plane and assign back to node
        for(int i = 0; i < N; i++)
		{
            Vector3 p = pointOnPlane( centers[i], plane_normal, plane_d );
			nmap[i]->vis_property[NODE_CENTER] = QVector3D( RotateFromTo(plane_normal, Vector3(0,0,1), p, plane_center) );
		}
            
        graph->property["embeded2D"] = true;
    }

	static void sphereRandomEmbed( Structure::Graph * graph )
	{
		// Get nodes centers
		QMap<int, Node*> nmap;
		std::vector<Vector3> centers;
		foreach(Node * n, graph->nodes)
		{
			nmap[nmap.size()] = n;

			double u = uniform(-1,1);
			double theta = uniform(0, 2 * M_PI);
			Vector3 randomCenter(sqrt(1 - (u*u)) * cos(theta), sqrt(1 - (u*u)) * sin(theta), u);

			centers.push_back( randomCenter );
		}
		int N = centers.size();

		// PCA
		Vector3 plane_center(0), plane_normal(0); Scalar plane_d = 0;
		planePCA(centers, plane_center, plane_normal, plane_d);

		// Project to PCA plane, to XY plane and assign back to node
		for(int i = 0; i < N; i++)
		{
			Vector3 p = pointOnPlane( centers[i], plane_normal, plane_d );
			nmap[i]->vis_property[NODE_CENTER] = QVector3D( RotateFromTo(plane_normal, Vector3(0,0,1), p, plane_center) );
		}

		graph->property["embeded2D"] = true;
	}

	static void circleEmbed( Structure::Graph * graph )
	{
		int N = graph->nodes.size();

		double theta = (2 * M_PI) / N;

		for(int i = 0; i < N; i++)
		{
			graph->nodes[i]->vis_property[NODE_CENTER] = QVector3D( cos(i * theta), sin(i * theta), 0 );
		}

		graph->property["embeded2D"] = true;
	}

	static double inline uniform(double a = 0.0, double b = 1.0){
		double len = b - a;
		return ((double)rand()/RAND_MAX) * len + a;
	}

    static void planePCA(const std::vector<Vector3> & points, Vector3 & center, Vector3 & normal, Scalar & d)
    {
        int N = points.size();

        Scalar	x, y, z, xx, yy, zz, xy, xz, yz,
            sumX (0), sumY (0), sumZ (0),
            sumX2(0), sumY2(0), sumZ2(0),
            sumXY(0), sumXZ(0), sumYZ(0);

        Vector3 mean(0);

        foreach(Vector3 p, points){
            x = p.x();	y = p.y();	z = p.z();
            sumX  += x / N		;	sumY  += y / N		;	sumZ  += z / N;
            sumX2 += x * x / N	;	sumY2 += y * y / N	;	sumZ2 += z * z / N;
            sumXY += x * y / N	;	sumXZ += x * z / N	;	sumYZ += y * z / N;
            mean += p;
        }

        mean /= points.size();

        xx = sumX2 - sumX * sumX; yy = sumY2 - sumY * sumY;	zz = sumZ2 - sumZ * sumZ;
        xy = sumXY - sumX * sumY; xz = sumXZ - sumX * sumZ;	yz = sumYZ - sumY * sumZ;

        Matrix3d covariance;
        covariance << xx, xy, xz,
                      xy, yy, yz,
                      xz, yz, zz;

        SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);
        Vector3d e_values = es.eigenvalues();;
        Matrix3d e_vectors = es.eigenvectors();

        Vector3d u = e_vectors.col(1).normalized();
        Vector3d v = e_vectors.col(2).normalized();
        Vector3d n = u.cross(v);

        // Output: a plane
        center  = mean;
        normal  = Vector3(n[0], n[1], n[2]);
        d       = -dot(normal, center);
    }

    static Vector3 pointOnPlane(Vector3 q, Vector3 pn, Scalar pd)
    {
        Scalar t = dot(pn, q) - pd;
        return q - t * pn;
    }

    static Vector3 RotateFromTo(Vector3 from, Vector3 to, Vector3 & point, Vector3 pivot = Vector3(0))
    {
		Vector3 axis = cross(from, to).normalized();
		double theta = acos( qRanged(-1.0, dot(from.normalize(), to.normalize()), 1.0) );

		point -= pivot;
		point = (point * cos(theta) + cross(axis, point) * sin(theta) + axis * dot(axis, point) * (1 - cos(theta)));
		point += pivot;

		return point;
    }

};
