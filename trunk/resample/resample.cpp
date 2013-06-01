#include "resample.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "SpherePackSampling.h"
#include "../CustomDrawObjects.h"
#include "PCA3.h"

#include "rimls.h"

#include <iostream>
#include <fstream>

ResampleWidget * rw = NULL;

void myresample::create()
{
	if(!rw) rw = new ResampleWidget(this);
	rw->show();
}

void myresample::doResample()
{
    drawArea()->deleteAllRenderObjects();

    int millions = 3e5;
    double s = 0.01;
	double r = mesh()->bbox().diagonal().norm() * s;

    PointSoup * ps = new PointSoup;
    PlaneSoup * ls = new PlaneSoup(r);

    std::vector<Vec3d> samples = SpherePackSampling::sample(mesh(), millions, r);

	// Add into a KD-tree
	NanoKdTree tree;
	foreach(Vec3d p, samples) tree.addPoint(p);
	tree.build();

    foreach(Vec3d p, samples)
	{
        ps->addPoint(p);
		
        // Collect neighbors
		int k = 16;
		KDResults matches;
		tree.k_closest(p, k, matches);
        std::vector<Vec3d> k_neighbors;
        foreach(KDResultPair i, matches) k_neighbors.push_back(samples[i.first]);

        // Compute Plane
        Vec3d c,n;
        PCA3::setPlane(k_neighbors, c, n);
        ls->addPlane(c,n);
	}

    drawArea()->addRenderObject(ls);
    drawArea()->addRenderObject(ps);
    drawArea()->updateGL();

    // DEBUG ====
    ofstream myfile;
    myfile.open ("sheet_point_cloud.xyz");
    foreach(Vec3d p, samples)
        myfile << p[0] << " " << p[1] << " " << p[2] << "\n";
    myfile.close();

	/// MLS - test
	/*Surface_mesh::Vertex_property<Vector3> points = mesh()->vertex_property<Vector3>(VPOINT);
	Surface_mesh::Vertex_property<Vector3> normals = mesh()->vertex_property<Vector3>(VNORMAL, Vector3(0));

	Surface_mesh::Vertex_property<Vector3> new_points = mesh()->vertex_property<Vector3>("v:projected", Vector3(0));
	Surface_mesh::Vertex_property<Vector3> new_normals = mesh()->vertex_property<Vector3>("v:projected", Vector3(0));

	RmlsSurface mls( mesh() );

	double FilterScale = 2.0;
	int MaxProjectionIters = 15;
	double ProjectionAccuracy = 0.0001;
	int MaxRefittingIters = 3;
	double SigmaN = 0.75;

	mls.setFilterScale( FilterScale );
	mls.setMaxProjectionIters( MaxProjectionIters );
	mls.setProjectionAccuracy( ProjectionAccuracy );
	mls.setMaxRefittingIters( MaxRefittingIters );
	mls.setSigmaN( SigmaN );

	foreach(Vertex v, mesh()->vertices())
	{
		new_points[v] = mls.project(points[v], new_normals[v]);
	}

	foreach(Vertex v, mesh()->vertices())
	{
		points[v] = new_points[v];
    }*/
}

void myresample::doParameterize()
{
	//LSCM mapper( mesh() );

	//PolygonSoup * ts = new PolygonSoup;

	//foreach(Face f, mesh()->faces())
	//{
	//	std::vector<Vertex> vface;
	//	Surface_mesh::Vertex_around_face_circulator vit = mesh()->vertices(f),vend=vit;
	//	do{ vface.push_back( Vertex(vit) ); } while(++vit != vend);

	//	Vec2d uv0 = mapper.tex_coord[vface[0]];
	//	Vec2d uv1 = mapper.tex_coord[vface[1]];
	//	Vec2d uv2 = mapper.tex_coord[vface[2]];

	//	QVector<Eigen::Vector3d> pts;
	//	pts.push_back( Vec3d(uv0[0],uv0[1],0) );
	//	pts.push_back( Vec3d(uv1[0],uv1[1],0) );
	//	pts.push_back( Vec3d(uv2[0],uv2[1],0) );

	//	ts->addPoly(pts);
	//}

	//drawArea()->addRenderObject(ts);
	//drawArea()->updateGL();
}

Q_EXPORT_PLUGIN(myresample)
