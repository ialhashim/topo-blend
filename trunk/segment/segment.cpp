#include "segment.h"
#include "SurfaceMeshHelper.h"
#include "StarlabDrawArea.h"

void segment::initParameters(RichParameterSet* pars){
	pars->addParam( new RichFloat("zero_threshold", 0.0f, "Zero threshold", "Zero threshold used for segmentation"));
	pars->addParam( new RichInt("face_grow", 0, "Face grow", "Num iterations to grow valid faces"));
	pars->addParam( new RichFloat("angle_threshold", 0, "Angle threshold", "Minimal valid angle"));

}

void segment::applyFilter(RichParameterSet* pars)
{
	// Clear debug artifacts
	drawArea()->deleteAllRenderObjects();

	// Create helper object to access points and edge lengths
	SurfaceMeshHelper h(mesh());
	Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);
	ScalarEdgeProperty elen = h.computeEdgeLengths();

	// Retrieve parameters
	double eps = pars->getFloat("zero_threshold");
	int face_grow = pars->getInt("face_grow");
	double theta = pars->getFloat("angle_threshold");

	// Face property
	Surface_mesh::Face_property<bool> fvalid = mesh()->add_face_property<bool>("f:valid", true);

	foreach(Edge e, mesh()->edges())
	{
		if(elen[e] <= eps)
		{
			Face f1 = mesh()->face(mesh()->halfedge(e,0));
			Face f2 = mesh()->face(mesh()->halfedge(e,1));
			fvalid[f1] = fvalid[f2] = false;
		}
	}

	for(int i = 0; i < face_grow; i++)
	{
		// Find valid vertices
		foreach(Vertex v, mesh()->vertices())
		{
			bool hasValidFace = false;

			// Check any adjacent valid faces
			Surface_mesh::Face_around_vertex_circulator fj(mesh(), v), fend = fj;
			while(++fj != fend){
				if(fvalid[fj]) {
					hasValidFace = true;
					break;
				}
			}

			// Spread found validity
			if(hasValidFace){
				fj = fend; // reset
				while(++fj != fend) 
					fvalid[fj] = true;
			}
		}
	}

	foreach(Face f, mesh()->faces())
	{
		// Get face points
		QVector<QVector3D> p;
		Surface_mesh::Vertex_around_face_circulator vit, vend;
		vit = vend = mesh()->vertices(f);
		do{ p.push_back(points[vit]); } while(++vit != vend);

		if(fvalid[f])
		{
			drawArea()->drawTriangle(p[0],p[1],p[2], Qt::blue);
			drawArea()->drawSegment(p[0],p[1], 1, Qt::blue);
			drawArea()->drawSegment(p[1],p[2], 1, Qt::blue);
			drawArea()->drawSegment(p[2],p[0], 1, Qt::blue);
		}
		else
		{
			drawArea()->drawTriangle(p[0],p[1],p[2], Qt::red);
			drawArea()->drawSegment(p[0],p[1], 3, Qt::red);
			drawArea()->drawSegment(p[1],p[2], 3, Qt::red);
			drawArea()->drawSegment(p[2],p[0], 3, Qt::red);
		}
	}
}

Q_EXPORT_PLUGIN(segment)
