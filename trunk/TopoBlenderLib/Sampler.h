#pragma once

#include "SurfaceMeshModel.h"
#include "SurfaceMeshHelper.h"

using namespace SurfaceMeshTypes;

// Helper structures
struct SamplePoint{
	Vec3d pos, n;
	double weight;
	double u,v;
	int findex; // index of sampled face
	int flag;

	SamplePoint(const Vec3d& position = Vec3d(), const Vec3d& normal = Vec3d(), 
		double Weight = 0.0, int face_index = -1.0, double U = 0.0, double V = 0.0, int flags = 0)
	{
		pos = position;
		n = normal;
		weight = Weight;
		findex = face_index;
		u = U;
		v = V;
		flag = flags;
	}
};

struct AreaFace{
	double area;
	Surface_mesh::Face f;

    AreaFace(double a = 0.0, Surface_mesh::Face face = Surface_mesh::Face()) : area(a), f(face){}

	bool operator< (const AreaFace & af) const { return area < af.area; }
	void setValue (double val) { area = val; }
};

enum SamplingMethod { FACE_CENTER, RANDOM_BARYCENTRIC };

// Class definition
class Sampler{

private:

public:
	
	SamplingMethod method;

    Sampler(SurfaceMeshModel * srcMesh = NULL, SamplingMethod samplingMethod = RANDOM_BARYCENTRIC );
	Sampler(void * srcMesh, SamplingMethod samplingMethod);
	// Get samples
	SamplePoint getSample(double weight = 0.0);
    std::vector<SamplePoint> getSamples(int numberSamples, double weight = 0.0);

    SurfaceMeshModel * mesh;
	double totalMeshArea;

	// For Monte Carlo
    std::vector<AreaFace> interval;
    ScalarFaceProperty farea;
    ScalarFaceProperty fprobability;
    Vector3FaceProperty fcenter;
    Vector3FaceProperty fnormal;
    Vector3VertexProperty points;

    Vector3 getBaryFace(Surface_mesh::Face f, double U, double V);
};

// Helper functions
double inline uniform(double a = 0.0, double b = 1.0){
    double len = b - a;
    return ((double)rand()/RAND_MAX) * len + a;
}

static inline void RandomBaricentric(double * interp){
	interp[1] = uniform();
	interp[2] = uniform();

	if(interp[1] + interp[2] > 1.0)
	{
		interp[1] = 1.0 - interp[1];
		interp[2] = 1.0 - interp[2];
	}

	interp[0] = 1.0 - (interp[1] + interp[2]);
}
