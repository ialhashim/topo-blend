#ifndef MORPHER_H
#define MORPHER_H

#include <QObject>
#include "geometry_morph.h"
#include "Octree.h"

using namespace Structure;

class Morpher : public QObject
{
	Q_OBJECT

public:

explicit	Morpher(SurfaceMeshModel *mesh1, SurfaceMeshModel *mesh2, 
	Graph graph1, Graph graph2, 
	QObject *parent=0);

	~Morpher();

	void get_faces(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Array2D_Vector3 &source_faces, Array2D_Vector3 &target_faces);
	void buildOctree(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Octree * &source_octree, Octree*  &target_octree, int numTrisPerNode=50);


	void generateInBetween(int numStep, int uResolution=10, int vResolution=10, int timeResolution=10, int thetaResolution=10, int phiResolution=6);

	// cylinder resampling
	std::vector<Vector3> resampleCoutourPoints(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
		double curr_t, int thetaResolution, double thetaRange, Vector3 &previous_startDirection, Octree* octree);

	std::vector<Vector3> resampleCoutourPoints3(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
		double curr_t, int thetaResolution, double thetaRange, Vector3 fixed_startDirection, Octree* octree);

	Array2D_Vector3 cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection,
		int timeResolution, int thetaResolution, double thetaRange, Octree* octree, bool SheetBoundary);
	
	// plane resampling, return upside and downside resampled faces
	std::vector<Array2D_Vector3> planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, int uResolution, int vResolution, Octree* octree);

	// sphere resampling
	Array2D_Vector3 sphereResampling(Array2D_Vector3 mesh_faces, Vector3 endPoint, Vector3 normalDirection, Vector3 startDirection,
		int thetaResolution, int phiResolution, double thetaRange, double phiRange, Octree *octree);

	// basic resamping action
	bool rayIntersectsTriangle(Vector3 origin, Vector3 direction, std::vector<Vector3> triangle,Vector3 &intersect_point, double &radius);

	
	//save options

	// only add plane, cylinder, sphere patches, not dealing with the boundaries
	void addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane,SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addCornerFaces(Array2D_Vector3 resampledSphere, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addEndFaces(Array2D_Vector3 resampledSphere, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase);

	// blending options
	void linear_Interp_Corrd2(SurfaceMeshModel &source, SurfaceMeshModel &target, int numStep);
	
	
	SurfaceMeshModel *source_mesh;
	SurfaceMeshModel *target_mesh;

	Structure::Graph source_graph;
	Structure::Graph target_graph;

	//std::vector<Vector3> source_vertices;
	Array2D_Vector3 source_faces;
	//std::vector<Vector3> target_vertices;
	Array2D_Vector3 target_faces;

	Octree* source_octree; 
	Octree* target_octree;

private:
	
};

#endif // MORPHER_H
