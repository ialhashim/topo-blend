#include "Sampler.h"

Sampler::Sampler(SurfaceMesh::Model * srcMesh, SamplingMethod samplingMethod)
{
	if(srcMesh == NULL) 
		return;
	else
		mesh = srcMesh;

	method = samplingMethod;
	
	mesh->update_face_normals();
	fnormal = mesh->get_face_property<Vector3>(FNORMAL);

	SurfaceMeshHelper h(mesh);
    farea = h.computeFaceAreas();
    points = h.getVector3VertexProperty(VPOINT);

    FaceBarycenterHelper fh(mesh);
    fcenter = fh.compute();

	// Sample based on method selected
	if( method == RANDOM_BARYCENTRIC )
	{
        // Compute all faces area
        fprobability = mesh->face_property<Scalar>("f:probability", 0);
		this->totalMeshArea = 0;

		Surface_mesh::Face_iterator fit, fend = mesh->faces_end();
		for (fit = mesh->faces_begin(); fit != fend; ++fit)
            totalMeshArea += farea[fit];

        for (fit = mesh->faces_begin(); fit != fend; ++fit)
            fprobability[fit] = farea[fit] / totalMeshArea;

        interval = std::vector<AreaFace>(mesh->n_faces() + 1);
        interval[0] = AreaFace(0.0, Surface_mesh::Face(0));
		int i = 0;

		// Compute mesh area in a cumulative manner
        foreach(Face f, mesh->faces())
		{
            interval[f.idx()+1] = AreaFace(interval[i].area + fprobability[f], f);
			i++;
		}
	}
	else if( method ==  FACE_CENTER )
	{
		// No preparations needed..
	}
}

SamplePoint Sampler::getSample(double weight)
{
	SamplePoint sp;
	double r;
	double b[3];

	if( method == RANDOM_BARYCENTRIC )
	{
		// r, random point in the area
		r = uniform();

		// Find corresponding face
        std::vector<AreaFace>::iterator it = lower_bound(interval.begin(), interval.end(), AreaFace(qMin(r,interval.back().area)));
        Surface_mesh::Face f = it->f;

		// Add sample from that face
		RandomBaricentric(b);

        sp = SamplePoint( getBaryFace(f, b[0], b[1]), fnormal[f], weight, f.idx(), b[0], b[1]);
	}
	else if( method ==  FACE_CENTER )
	{
		int fcount = mesh->n_faces();

		int randTriIndex = (int) (fcount * (((double)rand()) / (double)RAND_MAX)) ;

		if( randTriIndex >= fcount )
			randTriIndex = fcount - 1;

        Surface_mesh::Face f(randTriIndex);

		// Get triangle center and normal
        sp = SamplePoint(fcenter[f], fnormal[f], farea[f], f.idx(), 1 / 3.0, 1 / 3.0);
	}

	return sp;
}

std::vector<SamplePoint> Sampler::getSamples(int numberSamples, double weight)
{
    std::vector<SamplePoint> samples(numberSamples);

	for(int i = 0; i < numberSamples; i++)
	{
		samples[i] = getSample(weight);
	}

	return samples;
}

Vector3 Sampler::getBaryFace( Surface_mesh::Face f, double U, double V )
{
    QVector<Vector3> v;
    Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
    do{ v.push_back(points[vit]); } while(++vit != vend);

    if(U == 1.0) return v[1];
    if(V == 1.0) return v[2];

    double b1 = U;
    double b2 = V;
    double b3 = 1.0 - (U + V);

    Vector3 p;
    p.x() = (b1 * v[0].x()) + (b2 * v[1].x()) + (b3 * v[2].x());
    p.y() = (b1 * v[0].y()) + (b2 * v[1].y()) + (b3 * v[2].y());
    p.z() = (b1 * v[0].z()) + (b2 * v[1].z()) + (b3 * v[2].z());
    return p;
}
