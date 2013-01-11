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
	Graph graph1, Graph graph2, int uResolution, int vResolution, int timeResolution, int thetaResolution, int phiResolution,
	QObject *parent=0);

	~Morpher();

	void get_faces(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Array2D_Vector3 &source_faces, Array2D_Vector3 &target_faces);
	void buildOctree(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Octree * &source_octree, Octree*  &target_octree, int numTrisPerNode);
	
	// Experiment
	void testCase();

	// resampling options
	void resampling(int uResolution=10, int vResolution=10, 
		int timeResolution=10, int thetaResolution=10, int phiResolution=6);
	void curveResampling(NURBSCurve source_curve, NURBSCurve target_curve, int timeResolution, int thetaResolution, int phiResolution);
	void sheetResampling(NURBSRectangle source_sheet, NURBSRectangle target_sheet, int uResolution, int vResolution, int thetaResolution, int phiResolution);

	// cylinder resampling
	Array2D_Vector3 cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection,
		int timeResolution, int thetaResolution, double thetaRange, Octree* octree, bool SheetBoundary);
	std::vector<Vector3> resampleCoutourPointsCylinder(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
		double curr_t, int thetaResolution, double thetaRange, Vector3 &previous_startDirection, Octree* octree);
	std::vector<Vector3> resampleCoutourPointsPlane(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
		double curr_t, int thetaResolution, double thetaRange, Vector3 fixed_startDirection, Octree* octree);

	
	// plane resampling, return upside and downside resampled faces
	std::vector<Array2D_Vector3> planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, Vector3 initialDirection, int uResolution, int vResolution, Octree* octree);

	// sphere resampling
	Array2D_Vector3 sphereResampling(Array2D_Vector3 mesh_faces, Vector3 endPoint, Vector3 normalDirection, Vector3 startDirection,
		int thetaResolution, int phiResolution, double thetaRange, double phiRange, Octree *octree);

	// basic resamping action
	bool rayIntersectsTriangle(Vector3 origin, Vector3 direction, std::vector<Vector3> triangle,Vector3 &intersect_point, double &radius);

	
	//save options
	// only add plane, cylinder, sphere patches, not dealing with the boundaries
	void addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane,SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addCornerFaces(Array2D_Vector3 resampledSphere, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase);
	void addEndFaces(Array2D_Vector3 resampledSphere, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase);

	void stitchCylinder(int timeResolution, int thetaResolution, int sampling_phiResolution);
	void stitchPlane(int uResolution, int vResolution, int sampling_thetaResolution, int sampling_phiResolution);

	// blending options
	SurfaceMeshModel* generateInBetween(std::vector<SurfaceMeshModel*> source_models, std::vector<SurfaceMeshModel*> target_models, std::vector<double> T );
	
	SurfaceMeshModel *source_mesh;
	SurfaceMeshModel *target_mesh;

	Structure::Graph source_graph;
	Structure::Graph target_graph;

	Array2D_Vector3 source_faces;
	Array2D_Vector3 target_faces;

	Octree* source_octree; 
	Octree* target_octree;

	SurfaceMeshModel* resampledSourceMesh;
	SurfaceMeshModel* resampledTargetMesh;
	std::vector<Vertex> source_verticesIdx, target_verticesIdx;

	std::vector< std::vector<SurfaceMeshModel::Vertex> > facesToBuild;
	void buildFaces(SurfaceMeshModel *mesh);

	Vec3d intersectionPoint( Ray ray, Octree * useTree, int * faceIndex = NULL);

	std::vector<Vec3d> debugPoints,debugPoints2;

	SurfaceMeshModel* mergeVertices( SurfaceMeshModel * model, double threshold = 1e-6);

private:
	int uResolution; 
	int vResolution;
	int timeResolution; 
	int thetaResolution;
	int phiResolution;
};

#endif // MORPHER_H
