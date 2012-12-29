#ifndef MORPHER_H
#define MORPHER_H

#include <QObject>
#include "geometry_morph.h"


class Morpher : public QObject
{
	Q_OBJECT

public:
explicit Morpher(SurfaceMeshModel *mesh1, 
		SurfaceMeshModel *mesh2, 
		NURBSCurve curve1, 
		NURBSCurve curve2, 
		QObject *parent=0);

	Morpher(SurfaceMeshModel *mesh1, SurfaceMeshModel *mesh2, Structure::Graph graph1, Structure::Graph graph2);

	~Morpher();

	void get_faces(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, std::vector<std::vector<Vector3>> &source_faces, std::vector<std::vector<Vector3>> &target_faces);

	void generateInBetween(int numStep, int timeResolution, int angleResotuion);
	std::vector<std::vector<Vector3>> crossSectionResampling(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve, 
		int timeResolution, int angleResotuion);

	std::vector<std::vector<Vector3>> crossSectionResampling2(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve, 
		int timeResolution, int angleResolution, std::vector<std::vector<Vector3>> &sampleDirections, std::vector<std::vector<double>> &radii);

	std::vector<Vector3> resampleCoutourPoints(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve,
		double current_time, int angleResolution, Vector3 &previous_startDirection);

	std::vector<Vector3> resampleCoutourPoints2(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve,
		double current_time, int angleResolution, Vector3 &previous_startDirection, std::vector<Vector3> &oneRing_sampleDirections, std::vector<double> &oneRing_radii);

	bool rayIntersectsTriangle(Vector3 origin, Vector3 direction, std::vector<Vector3> triangle,Vector3 &intersect_point, double &radius);

	void saveQuadMesh(std::string filename, std::vector<std::vector<Vector3>> crossSections);
	void saveTriMesh(std::string filename, std::vector<std::vector<Vector3>> crossSections);

	SurfaceMeshModel* mesh_from_crossSections(std::vector<std::vector<Vector3>> blend_crossSections);

	void align_sampleStartDirection(std::vector<std::vector<Vector3>> &source_crossSections, std::vector<std::vector<Vector3>> &target_crossSections,
		std::vector<std::vector<Vector3>> &souce_sampleDirections, std::vector<std::vector<Vector3>> &target_sampleDirections,
		std::vector<std::vector<double>> souce_radii, std::vector<std::vector<double>> target_radii);
	void linear_Interp_Corrd(std::vector<std::vector<Vector3>> source_crossSections, std::vector<std::vector<Vector3>> target_crossSections, int numStep);
	void linear_Interp_Radius(std::vector<std::vector<Vector3>> souce_sampleDirections, std::vector<std::vector<Vector3>> target_sampleDirections, 
		std::vector<std::vector<double>> souce_radii, std::vector<std::vector<double>> radii, int numStep);
	
	
	SurfaceMeshModel *source_mesh;
	SurfaceMeshModel *target_mesh;
	NURBSCurve source_curve;
	NURBSCurve target_curve;
	Structure::Graph source_graph;
	Structure::Graph target_graph;

	//std::vector<Vector3> source_vertices;
	std::vector<std::vector<Vector3>> source_faces;
	//std::vector<Vector3> target_vertices;
	std::vector<std::vector<Vector3>> target_faces;


private:
	
};

#endif // MORPHER_H
