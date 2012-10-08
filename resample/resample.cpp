#include "resample.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "SpherePackSampling.h"
#include "../segment/CustomDrawObjects.h"

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

    /*int millions = 1e4;
    double s = 0.01;
	double r = mesh()->bbox().size().length() * s;

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
		KDResults matches;
		tree.k_closest(p, 16, matches);

        std::vector<Vec3d> k_neighbors;
        foreach(KDResultPair i, matches) k_neighbors.push_back(samples[i.first]);

        // Plane
        Vec3d c,n;
        PCA3::setPlane(k_neighbors, c, n);
        ls->addPlane(c,n);
	}

    drawArea()->addRenderObject(ls);
    drawArea()->addRenderObject(ps);
    drawArea()->updateGL();

    // DEBUG ====
    /*ofstream myfile;
    myfile.open ("sheet_point_cloud.pts");
    foreach(Vec3d p, samples)
        myfile << "v " << p << "\n";
    myfile.close();*/

	Surface_mesh::Vertex_property<Vector3> points = mesh()->vertex_property<Vector3>(VPOINT);
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
	}
}

Q_EXPORT_PLUGIN(myresample)
