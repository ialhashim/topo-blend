#include "resample.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "SpherePackSampling.h"
#include "../segment/CustomDrawObjects.h"
#include "PCA3.h"
#include "ball_pivoting.h"

#include <iostream>
#include <fstream>

void myresample::create()
{
    rw = new ResampleWidget(this);
    rw->show();
}

void myresample::doResample()
{
    drawArea()->deleteAllRenderObjects();

    int millions = 1e4;
    double s = 0.01;
	double r = mesh()->bbox().size().length() * s;

    PointSoup * ps = new PointSoup;
    PlaneSoup * ls = new PlaneSoup(r);

    std::vector<Vec3d> samples = SpherePackSampling::sample(mesh(), millions, r);

	// Add into a KD-tree
	std::vector< vector<double> > tpoints;
	foreach(Vec3d p, samples) tpoints.push_back( SpherePackSampling::toKDPoint(p) );
	KDTree tree(tpoints);

    foreach(Vec3d p, samples)
	{
        ps->addPoint(p);
		
        // Collect neighbors
        std::vector<int> idxs;
        std::vector<double> d;
        tree.k_closest_points( SpherePackSampling::toKDPoint(p), 16, idxs, d );

        std::vector<Vec3d> k_neighbors;
        foreach(int i, idxs) k_neighbors.push_back(samples[i]);

        // Plane
        Vec3d c,n;
        PCA3::setPlane(k_neighbors, c, n);
        ls->addPlane(c,n);
	}

    drawArea()->addRenderObject(ls);
    drawArea()->addRenderObject(ps);
    drawArea()->setRenderer(mesh(), "Bounding Box");
    drawArea()->updateGL();

    // DEBUG ====
    /*ofstream myfile;
    myfile.open ("sheet_point_cloud.pts");
    foreach(Vec3d p, samples)
        myfile << "v " << p << "\n";
    myfile.close();*/
}

void myresample::doBallPivoting()
{
	SurfaceMeshModel * m = new SurfaceMeshModel("", "ball_pivot");

	// Position
	Surface_mesh::Vertex_property<Vector3> points = mesh()->get_vertex_property<Vector3>(VPOINT);
	foreach(Vertex v, mesh()->vertices())
		m->add_vertex(points[v]);

	// Normal
	//Surface_mesh::Vertex_property<Vector3> normals = mesh()->vertex_property<Vector3>(VNORMAL);
	//Surface_mesh::Vertex_property<Vector3> n = m->vertex_property<Vector3>(VNORMAL);
	//foreach(Vertex v, mesh()->vertices())
	//	n[v] = normals[v];

	// Ball Pivoting
	BallPivoting ball(m);
	ball.BuildMesh();

	document()->pushBusy();
	m->updateBoundingBox();
	document()->addModel(m);
	document()->popBusy();
}

Q_EXPORT_PLUGIN(myresample)
