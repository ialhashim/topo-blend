#include <QFile>

// Experiments
#include "NanoKdTree.h"
int missCount = 0;

#include "Morpher.h"
#include "surface_mesh/IO.h"
#include "surface_mesh/IO_off.cpp"

#define SIGN(x) (((x) < 0) ? (-1) : (((x) > 0) ? 1 : 0))
#define ROTATE_VEC(u, v, theta, axis) (u = v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)))
#define VEC_FROM_POINTS(a,b,c) \
	(a)[0] = (b)[0] - (c)[0];	\
	(a)[1] = (b)[1] - (c)[1];	\
	(a)[2] = (b)[2] - (c)[2];
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#define USE_OCTREE 1

Morpher::Morpher(SurfaceMeshModel *mesh1, SurfaceMeshModel *mesh2, Graph graph1, Graph graph2,	
	int uResolution, int vResolution, int timeResolution, int thetaResolution, int phiResolution, QObject *parent): QObject(parent)
{
	source_mesh = mesh1;
	target_mesh = mesh2;

	source_graph = graph1;
	target_graph = graph2;

	resampledSourceMesh = new SurfaceMeshModel("resampled_source.off","resampled_source");
	resampledTargetMesh = new SurfaceMeshModel("resampled_target.off","resampled_target");

	get_faces(source_mesh, target_mesh, source_faces, target_faces);
	buildOctree(source_mesh, target_mesh, source_octree, target_octree, 20);

	resampling(uResolution, vResolution, timeResolution, thetaResolution, phiResolution);
}

Morpher::~Morpher()
{

}

void Morpher::get_faces(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Array2D_Vector3 &source_faces, Array2D_Vector3 &target_faces)
{
	Vector3VertexProperty source_points = source_mesh->vertex_property<Vector3>(VPOINT);
	Vector3VertexProperty target_points = target_mesh->vertex_property<Vector3>(VPOINT);
	// Go over faces
	foreach(Face f, source_mesh->faces())
	{
		// Collect its points
		Surface_mesh::Vertex_around_face_circulator vit = source_mesh->vertices(f),vend=vit;
		std::vector<Vector3> face_vertices;
		do{ face_vertices.push_back(source_points[vit]); } while(++vit != vend);
		source_faces.push_back(face_vertices);
	}

	// Go over faces
	foreach(Face f, target_mesh->faces())
	{
		// Collect its points
		Surface_mesh::Vertex_around_face_circulator vit = target_mesh->vertices(f),vend=vit;
		std::vector<Vector3> face_vertices;
		do{ face_vertices.push_back(target_points[vit]); } while(++vit != vend);
		target_faces.push_back(face_vertices);
	}
}

void Morpher::buildOctree(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, Octree* &source_octree, Octree* &target_octree, int numTrisPerNode/* =50 */)
{
	source_octree = NULL;
	target_octree = NULL;

	if(USE_OCTREE)
	{
		std::vector<Surface_mesh::Face> allTris1, allTris2;
		foreach(Face f, source_mesh->faces()) allTris1.push_back(f);
		foreach(Face f, target_mesh->faces()) allTris2.push_back(f);

		source_octree = new Octree(numTrisPerNode, source_mesh);
		target_octree=new Octree(numTrisPerNode, target_mesh);

		source_octree->initBuild(allTris1, numTrisPerNode);
		target_octree->initBuild(allTris2, numTrisPerNode);
	}
}

void Morpher::resampling(int uResolution, int vResolution, int timeResolution, int thetaResolution, int phiResolution)
{
	// transverse the graph nodes
	for(int i=0;i<source_graph.nodes.size();i++)
	{
		// do cross section resampling to curve nodes
		if(source_graph.nodes[i]->type()==Structure::CURVE && target_graph.nodes[i]->type()==Structure::CURVE)
		{
			Curve* sourceCurve = (Curve *)source_graph.nodes[i];
			Curve* targetCurve = (Curve *)target_graph.nodes[i];
			NURBSCurve source_curve = sourceCurve->curve;
			NURBSCurve target_curve = targetCurve->curve;

			curveResampling(source_curve, target_curve, timeResolution,thetaResolution, phiResolution);
		}

		if(source_graph.nodes[i]->type()==Structure::SHEET && target_graph.nodes[i]->type()==Structure::SHEET)
		{
			Sheet* sourceSheet = (Sheet*) source_graph.nodes[i];
			Sheet* targetSheet = (Sheet*) target_graph.nodes[i];
			NURBSRectangle source_sheet = sourceSheet->surface;
			NURBSRectangle target_sheet = targetSheet->surface;

			sheetResampling(source_sheet, target_sheet, uResolution, vResolution, thetaResolution, phiResolution);
		}
	}
}

void Morpher::curveResampling(NURBSCurve source_curve, NURBSCurve target_curve, int timeResolution, int thetaResolution, int phiResolution )
{	
	Vector3 initialSourceDirection, initialTargetDirection;
	Vector3 initialSourceTangent, initialTargetTangent;
	double thetaRange = 2*M_PI;
	double phiRange = M_PI/2;
	int sampling_thetaResolution = (int)thetaResolution*thetaRange/(2*M_PI);
	int sampling_phiResolution = (int)phiResolution*phiRange/(M_PI);   // full range of phi here is M_PI

	// set the initial theta sampling direction as the curve binormal
	initialSourceDirection=source_curve.GetBinormal(0);
	initialTargetDirection=target_curve.GetBinormal(0);

	initialSourceTangent=source_curve.GetTangent(0);
	initialTargetTangent=target_curve.GetTangent(0);

	// align initial sampling direction
	// set the initial sampling direction as the 
	if(dot(initialSourceDirection,initialTargetDirection)<0)
		initialTargetDirection=-1*initialTargetDirection;

	if (dot(initialSourceTangent,initialTargetTangent)<0)
	{
		initialTargetTangent=-initialTargetTangent;
	}

	if(dot(initialSourceDirection, initialTargetDirection)<0.5)
	{
		initialTargetDirection=initialSourceDirection;
		Vector3 targetK=cross(initialTargetDirection, initialTargetTangent);
		initialTargetDirection=cross(initialTargetTangent, targetK);
	}

	//assert(dot(initialSourceDirection,initialTargetDirection)>0.5);

	Array2D_Vector3 source_crossSections=cylinderResampling(source_faces,source_curve,initialSourceDirection,timeResolution,sampling_thetaResolution,thetaRange, source_octree, false);
	Array2D_Vector3 target_crossSections=cylinderResampling(target_faces,target_curve,initialTargetDirection,timeResolution,sampling_thetaResolution,thetaRange, target_octree, false);

	int idxBase=source_verticesIdx.size();
	addCylinderFaces(source_crossSections, resampledSourceMesh, source_verticesIdx, idxBase);
	addCylinderFaces(target_crossSections, resampledTargetMesh, target_verticesIdx,idxBase);

	int tNum=source_crossSections.size();
	int thetaNum=source_crossSections[0].size();

	for(int curr_t=0; curr_t<tNum-1;curr_t++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[curr_t*thetaNum+thetaNum-1+idxBase]);
		face_vertex_idx.push_back(source_verticesIdx[curr_t*thetaNum+idxBase]);
		face_vertex_idx.push_back(source_verticesIdx[(curr_t+1)*thetaNum+idxBase]);
		face_vertex_idx.push_back(source_verticesIdx[(curr_t+1)*thetaNum+thetaNum-1+idxBase]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}


	Vector3 source_pos, sourceTangent, sourceBinormal, sourceNormal, thetaStartSource;
	Vector3 target_pos, targetTangent, targetBinormal, targetNormal, thetaStartTarget;

	// t=0
	source_pos=source_curve.GetPosition(0);
	sourceTangent=source_curve.GetTangent(0);
	//sourceNormal=source_curve.GetNormal(0);
	//sourceBinormal=source_curve.GetBinormal(0);


	target_pos=target_curve.GetPosition(0);
	targetTangent=target_curve.GetTangent(0);
	//targetNormal=target_curve.GetNormal(0);
	//targetBinormal=target_curve.GetBinormal(0);

	assert(dot(sourceTangent,targetTangent)>0.5);

	thetaStartSource=cross(initialSourceDirection, sourceTangent);
	thetaStartTarget=cross(initialTargetDirection, targetTangent);

	// in sphere sampling, the meaning of theta changes:
	// the theta in the cylinder represents the phi in the sphere
	// in sphere: theta=longitude, phi=latitude

	Array2D_Vector3 source_endResamplings1 = sphereResampling(source_faces,source_pos, thetaStartSource, -sourceTangent,
		sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, source_octree);
	Array2D_Vector3 target_endResamplings1 = sphereResampling(target_faces,target_pos, thetaStartTarget, -targetTangent,
		sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, target_octree);

	addEndFaces(source_endResamplings1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addEndFaces(target_endResamplings1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	// t=1
	source_pos=source_curve.GetPosition(1);
	sourceTangent=source_curve.GetTangent(1);
	//sourceNormal=source_curve.GetNormal(1);

	target_pos=target_curve.GetPosition(1);
	targetTangent=target_curve.GetTangent(1);
	//targetNormal=target_curve.GetNormal(1);

	assert(dot(sourceTangent,targetTangent) > 0.5);
	thetaStartSource = -cross(initialSourceDirection, sourceTangent);
	thetaStartTarget = -cross(initialTargetDirection, targetTangent);

	Array2D_Vector3 source_endResamplings2 = sphereResampling(source_faces,source_pos, thetaStartSource, sourceTangent,
		sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, source_octree);
	Array2D_Vector3 target_endResamplings2 = sphereResampling(target_faces,target_pos, thetaStartTarget, targetTangent,
		sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, target_octree);

	addEndFaces(source_endResamplings2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addEndFaces(target_endResamplings2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	stitchCylinder(timeResolution, thetaResolution, sampling_phiResolution);
}

void Morpher::sheetResampling(NURBSRectangle source_sheet, NURBSRectangle target_sheet, int uResolution, int vResolution, int thetaResolution, int phiResolution )
{
	QElapsedTimer resamplingTimer; resamplingTimer.start();

	// align initial sampling direction
	Vector3 position, sheetNormal, tangentU, tangentV;
	Vector3 initialSourceDirection, initialTargetDirection;
	Vector3 source_origin, target_origin;
	source_sheet.GetFrame(0, 0, source_origin, tangentU, tangentV, sheetNormal);
	initialSourceDirection=sheetNormal;

	target_sheet.GetFrame(0, 0, target_origin, tangentU, tangentV, sheetNormal);
	initialTargetDirection=sheetNormal;

	if (dot(initialSourceDirection, initialTargetDirection)<0)
	{
		initialTargetDirection=-initialTargetDirection;
	}

	std::vector<Array2D_Vector3> resampledSoucePlane=planeResamping(source_faces,source_sheet,initialSourceDirection,uResolution,vResolution, source_octree);
	std::vector<Array2D_Vector3> resampledTargetPlane=planeResamping(target_faces,target_sheet,initialTargetDirection,uResolution,vResolution, target_octree);

	addPlaneFaces(resampledSoucePlane, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addPlaneFaces(resampledTargetPlane, resampledTargetMesh,target_verticesIdx, target_verticesIdx.size());

	double t1=resamplingTimer.elapsed();
	qDebug() << QString("Plane resampling =%1 ms").arg(t1);

	QElapsedTimer cylinderTimer; cylinderTimer.start();

	// Boundary curve initialization
	//Vector3 initialSourceDirection, initialTargetDirection;
	double thetaRange=M_PI;
	double phiRange=M_PI/2;
	int sampling_thetaResolution=(int)thetaResolution*thetaRange/(2*M_PI);
	int sampling_phiResolution= (int)phiResolution*phiRange/(2*M_PI);


	//Vector3 position, sheetNormal, tangentU, tangentV;
	//	 set the initial theta sampling direction as the curve normal
	//source_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
	//initialSourceDirection=sheetNormal;

	//target_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
	//initialTargetDirection=sheetNormal;
	//assert(dot(initialSourceDirection,initialTargetDirection)>0.5);

	std::vector<Vector3> source_ctrlPoints1=source_sheet.GetControlPointsU(0);
	std::vector<Real> source_ctrlWeights1(source_ctrlPoints1.size(), 1.0);
	std::vector<Vector3> target_ctrlPoints1=target_sheet.GetControlPointsU(0);
	std::vector<Real> target_ctrlWeights1(target_ctrlPoints1.size(), 1.0);
	NURBSCurve source_curve1=NURBSCurve(source_ctrlPoints1, source_ctrlWeights1, 3, false, true);
	NURBSCurve target_curve1=NURBSCurve(target_ctrlPoints1, target_ctrlWeights1, 3, false, true);

	std::vector<Vector3> source_ctrlPoints2=source_sheet.GetControlPointsV(source_sheet.mNumVCtrlPoints-1);
	std::vector<Real> source_ctrlWeights2(source_ctrlPoints2.size(), 1.0);
	std::vector<Vector3> target_ctrlPoints2=target_sheet.GetControlPointsV(target_sheet.mNumVCtrlPoints-1);
	std::vector<Real> target_ctrlWeights2(target_ctrlPoints2.size(), 1.0);
	NURBSCurve source_curve2=NURBSCurve(source_ctrlPoints2, source_ctrlWeights2, 3, false, true);
	NURBSCurve target_curve2=NURBSCurve(target_ctrlPoints2, target_ctrlWeights2, 3, false, true);

	std::vector<Vector3> source_ctrlPoints3=source_sheet.GetControlPointsU(source_sheet.mNumUCtrlPoints-1);
	std::vector<Real> source_ctrlWeights3(source_ctrlPoints3.size(), 1.0);
	std::vector<Vector3> target_ctrlPoints3=target_sheet.GetControlPointsU(source_sheet.mNumUCtrlPoints-1);
	std::vector<Real> target_ctrlWeights3(target_ctrlPoints3.size(), 1.0);
	NURBSCurve source_curve3=NURBSCurve(source_ctrlPoints3, source_ctrlWeights3, 3, false, true);
	NURBSCurve target_curve3=NURBSCurve(target_ctrlPoints3, target_ctrlWeights3, 3, false, true);

	std::vector<Vector3> source_ctrlPoints4=source_sheet.GetControlPointsV();
	std::vector<Real> source_ctrlWeights4(source_ctrlPoints4.size(), 1.0);
	std::vector<Vector3> target_ctrlPoints4=target_sheet.GetControlPointsV();
	std::vector<Real> target_ctrlWeights4(target_ctrlPoints4.size(), 1.0);
	NURBSCurve source_curve4=NURBSCurve(source_ctrlPoints4, source_ctrlWeights4, 3, false, true);
	NURBSCurve target_curve4=NURBSCurve(target_ctrlPoints4, target_ctrlWeights4, 3, false, true);

	double ct1=cylinderTimer.elapsed();
	qDebug() << "	"<<QString("boundary cylinder initialization =%1ms").arg(ct1);

	// Start cylinder sampling at the boundaries
	// sampling using the sheet boundaries, counter-clockwise
	// curve1: transverse u while v=0
	Array2D_Vector3 source_crossSections1=cylinderResampling(source_faces,source_curve1,initialSourceDirection,
		uResolution,sampling_thetaResolution,thetaRange, source_octree, true);
	Array2D_Vector3 target_crossSections1=cylinderResampling(target_faces,target_curve1,initialTargetDirection,
		uResolution,sampling_thetaResolution, thetaRange, target_octree, true);

	// curve2: transverse v while u=uResolution
	Array2D_Vector3 source_crossSections2=cylinderResampling(source_faces,source_curve2,initialSourceDirection,
		vResolution,sampling_thetaResolution,thetaRange, source_octree, true);
	Array2D_Vector3 target_crossSections2=cylinderResampling(target_faces,target_curve2,initialTargetDirection,
		vResolution,sampling_thetaResolution, thetaRange, target_octree, true);

	// curve3: transverse u while v=vResolution
	initialSourceDirection= -initialSourceDirection;
	initialTargetDirection=-initialTargetDirection;
	Array2D_Vector3 source_crossSections3=cylinderResampling(source_faces,source_curve3,initialSourceDirection,
		uResolution,sampling_thetaResolution,thetaRange, source_octree,true);
	Array2D_Vector3 target_crossSections3=cylinderResampling(target_faces,target_curve3,initialTargetDirection,
		uResolution,sampling_thetaResolution, thetaRange,target_octree,true);

	// curve4: transverse v while u=0
	//initialSourceDirection=-initialSourceDirection;
	//initialTargetDirection= -initialTargetDirection;
	Array2D_Vector3 source_crossSections4=cylinderResampling(source_faces,source_curve4,initialSourceDirection,
		vResolution,sampling_thetaResolution,thetaRange,source_octree,true);
	Array2D_Vector3 target_crossSections4=cylinderResampling(target_faces,target_curve4,initialTargetDirection,
		vResolution,sampling_thetaResolution, thetaRange,target_octree,true);

	double ct2=cylinderTimer.elapsed();

	// add faces to the mesh
	addCylinderFaces(source_crossSections1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCylinderFaces(source_crossSections2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCylinderFaces(source_crossSections3, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCylinderFaces(source_crossSections4, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());

	addCylinderFaces(target_crossSections1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());
	addCylinderFaces(target_crossSections2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());
	addCylinderFaces(target_crossSections3, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());
	addCylinderFaces(target_crossSections4, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	double ct3=cylinderTimer.elapsed();
	qDebug() << "	"<<QString("cylinder resampling =%1 ms add face = %2ms").arg(ct2-ct1).arg(ct3-ct2);

	double t2=resamplingTimer.elapsed();
	qDebug() << QString("Boundary cylinder resampling =%1 ms").arg(t2-t1);

	// corner1: u=0, v=0
	source_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
	initialSourceDirection=sheetNormal;

	Array2D_Vector3 source_cornerResamplings1=sphereResampling(source_faces,position,-tangentV,initialSourceDirection,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, source_octree);

	target_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
	initialTargetDirection=sheetNormal;
	if (dot(initialSourceDirection, initialTargetDirection)<0)
	{
		initialTargetDirection=-initialTargetDirection;
	}

	Array2D_Vector3 target_cornerResamplings1=sphereResampling(target_faces,position,-tangentV,initialTargetDirection,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, target_octree);

	addCornerFaces(source_cornerResamplings1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCornerFaces(target_cornerResamplings1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	// corner2: u=1, v=0
	source_sheet.GetFrame(1,0,position, tangentU, tangentV, sheetNormal);
	if (dot(initialSourceDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 source_cornerResamplings2=sphereResampling(source_faces,position,tangentU,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, source_octree);

	target_sheet.GetFrame(1,0,position, tangentU, tangentV, sheetNormal);
	if (dot(initialTargetDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 target_cornerResamplings2=sphereResampling(target_faces,position,tangentU,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, target_octree);

	addCornerFaces(source_cornerResamplings2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCornerFaces(target_cornerResamplings2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	// corner3: u=1, v=1
	source_sheet.GetFrame(1,1,position, tangentU, tangentV, sheetNormal);
	if (dot(initialSourceDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 source_cornerResamplings3=sphereResampling(source_faces,position,tangentV,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, source_octree);

	target_sheet.GetFrame(1,1,position, tangentU, tangentV, sheetNormal);
	if (dot(initialTargetDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 target_cornerResamplings3=sphereResampling(target_faces,position,tangentV,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, target_octree);

	addCornerFaces(source_cornerResamplings3, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCornerFaces(target_cornerResamplings3, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	// corner4: u=0, v=1
	source_sheet.GetFrame(0,1,position, tangentU, tangentV, sheetNormal);
	if (dot(initialSourceDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 source_cornerResamplings4=sphereResampling(source_faces,position,-tangentU,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, source_octree);

	target_sheet.GetFrame(0,1,position, tangentU, tangentV, sheetNormal);
	if (dot(initialTargetDirection, sheetNormal)<0)
	{
		sheetNormal=-sheetNormal;
	}
	Array2D_Vector3 target_cornerResamplings4=sphereResampling(target_faces,position,-tangentU,sheetNormal,
		sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, target_octree);

	addCornerFaces(source_cornerResamplings4, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
	addCornerFaces(target_cornerResamplings4, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

	stitchPlane(uResolution,vResolution,sampling_thetaResolution,sampling_phiResolution);

    //double t3=resamplingTimer.elapsed();
	//qDebug() << QString("Corner resampling =%1 ms").arg(t3-t2);
}

Array2D_Vector3 Morpher::cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection,
	int timeResolution, int thetaResolution, double thetaRange, Octree* octree, bool SheetBoundary)
{
	double delta_t=1.0/timeResolution;

	std::vector<Vector3> cross_section;
	Array2D_Vector3 crossSections;

	// type check
	// cylinder sampling for plane (open, thetaSamplings=theResolution+1), and sampling direction fixed as the sheet normal
	if(SheetBoundary)
	{
		for (int i=1;i<timeResolution;i++)
		{
			cross_section=resampleCoutourPointsPlane(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);
			crossSections.push_back(cross_section);
		}
	}

	// for a true cylinder (close, thetaSamplings=theResolution), and sampling direction computed as the binormal each time
	else
	{
		for (int i=1;i<timeResolution;i++)
		{
			cross_section=resampleCoutourPointsPlane(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);
			crossSections.push_back(cross_section);
		}
	}
	return crossSections;
}


// for a true cylinder (close, thetaSamplings=theResolution), and sampling direction computed as the binormal each time
// status: octree work
// current: using octree
std::vector<Vector3> Morpher::resampleCoutourPointsCylinder(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
	double curr_t, int thetaResolution,double thetaRange, Vector3 &previous_startDirection, Octree* octree)
{
	double delta_theta=thetaRange/thetaResolution;
	double radius;

	if (thetaRange!=2*M_PI)
	{
		thetaResolution+=1;
	}

	std::vector<Vector3> intersections;

	Vector3 current_position;
	Vector3 curveTanget; // tangent of curve
	Vector3 sample_startDirection; // binormal of curve
	Vector3 biNormal;
	Vector3 curveNormal;

	pathCurve.GetFrame(curr_t,current_position,curveTanget,curveNormal,biNormal);

	sample_startDirection=biNormal;

	// at some time points, the frame may change its directions
	// in this situation, check the sample direction with the previous sample directions
	if(dot(previous_startDirection,sample_startDirection)<0)
	{
		sample_startDirection=-1*sample_startDirection;
		//curveTanget=-1*curveTanget;
	}

	previous_startDirection=sample_startDirection;

	Vector3 sampleDirection=sample_startDirection;

	for (int i=0;i<thetaResolution;i++)
	{
		Vector3 intersect_point;
		double rotate_theta=i*delta_theta;

		sampleDirection=ROTATE_VEC(sampleDirection, sample_startDirection, rotate_theta, curveTanget);

		if(USE_OCTREE)
		{
			intersections.push_back( intersectionPoint( Ray(current_position,sampleDirection), octree ) );
		}
		else
		{
			// Full test of all triangles
			for(int i=0;i<(int)mesh_faces.size();i++)
			{
				if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
				{
					intersections.push_back(intersect_point);
					break;
				}
			}
		}
	}

	return intersections;
}


// Fixed sampling direction to avoid skew
// cylinder sampling for plane (open, thetaSamplings=theResolution+1), and sampling direction fixed as the sheet normal
std::vector<Vector3> Morpher::resampleCoutourPointsPlane(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
	double curr_t, int thetaResolution, double thetaRange, Vector3 fixed_startDirection, Octree* octree)
{
	double delta_theta=thetaRange/thetaResolution;

	double radius;
	std::vector<Vector3> intersections;

	Vector3 current_position;
	Vector3 curveTanget; // tangent of curve
	Vector3 biNormal;
	Vector3 curveNormal;

	pathCurve.GetFrame(curr_t,current_position,curveTanget,curveNormal,biNormal);

	if (thetaRange!=2*M_PI)
	{
		thetaResolution+=1;
	}

	for (int i=0;i<thetaResolution;i++)
	{
		Vector3 intersect_point;
		double rotate_theta=i*delta_theta;

		Vector3 sampleDirection=ROTATE_VEC(sampleDirection, fixed_startDirection, rotate_theta, curveTanget);

		if (USE_OCTREE)
		{
			intersections.push_back( intersectionPoint( Ray(current_position,sampleDirection), octree ) );
		}
		else
		{
			// test for all faces
			for(int i=0;i<(int)mesh_faces.size();i++)
			{
				if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
				{
					intersections.push_back(intersect_point);
					break;
				}
			}
		}
	}

	return intersections;
}

// status: octree work
// current: using octree
std::vector<Array2D_Vector3> Morpher::planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, Vector3 initialDirection, int uResolution, int vResolution, Octree* octree)
{
	Array2D_Vector3 upsidePlane, downsidePlane;
	std::vector<Array2D_Vector3> resampledPlane;

	double delta_u=1.0/uResolution, delta_v=1.0/vResolution;

	for(int v_idx=1;v_idx<vResolution;v_idx++)
	{
		Vector3 sheetPoint;
		Vector3 uDirection, vDirection, sheetNormal;

		Array1D_Vector3 upIntersections, downIntersections;

		for(int u_idx=1;u_idx<uResolution;u_idx++)
		{

			sheet.GetFrame(u_idx*delta_u, v_idx*delta_v, sheetPoint, uDirection, vDirection, sheetNormal);

			if (dot(sheetNormal, initialDirection)<0)
			{
				sheetNormal=-sheetNormal;
			}

			if (USE_OCTREE)
			{
				// Up
				upIntersections.push_back( intersectionPoint( Ray(sheetPoint,sheetNormal), octree ) );

				// Down
				downIntersections.push_back( intersectionPoint( Ray(sheetPoint,-sheetNormal), octree ) );
			}
			else
			{
				Vec3d intersect_point;
				double radius = 0;

				// test for all faces
				for(int i=0;i<(int)mesh_faces.size();i++)
				{
					if(rayIntersectsTriangle(sheetPoint,sheetNormal, mesh_faces[i],intersect_point,radius))	{
						upIntersections.push_back(intersect_point);
						break;
					}
				}

				for(int i=0;i<(int)mesh_faces.size();i++)
				{
					if(rayIntersectsTriangle(sheetPoint,-sheetNormal, mesh_faces[i],intersect_point,radius)){
						downIntersections.push_back(intersect_point);
						break;
					}
				}
			}
		}
		upsidePlane.push_back(upIntersections);
		downsidePlane.push_back(downIntersections);
	}


	resampledPlane.push_back(upsidePlane);
	resampledPlane.push_back(downsidePlane);

	return resampledPlane;
}


// phiAxis is set to the sampling start direction, the first intersection is also the pole of the sphere
// first rotate theta around thetaAxis, at a fixed theta, rotate phi around phiAxis
Array2D_Vector3 Morpher::sphereResampling(Array2D_Vector3 mesh_faces, Vector3 endPoint, 
	Vector3 thetaAxis, Vector3 phiAxis, 
	int thetaResolution, int phiResolution, 
	double thetaRange, double phiRange, Octree *octree)
{

	Array2D_Vector3 sphereResamplings;

	double delta_phi=phiRange/phiResolution;
	double delta_theta=thetaRange/thetaResolution;
	double radius;

	Vector3 startDirection;
	Vector3 intersect_point;

	if (thetaRange!=2*M_PI)
	{
		thetaResolution++;
	}

	if (phiRange!=2*M_PI)
	{
		phiResolution++;
	}

	for (int curr_theta=0; curr_theta< thetaResolution; curr_theta++)
	{
		std::vector<Vector3> phiResamplings;
		double rotate_theta=curr_theta*delta_theta;

		startDirection=phiAxis;
		Vector3 sampleDirection=ROTATE_VEC(sampleDirection, startDirection, rotate_theta, thetaAxis);

		// set startDirection for phi sampling
		startDirection=sampleDirection;

		for (int curr_phi=0; curr_phi<phiResolution; curr_phi++)
		{
			double rotate_phi= curr_phi*delta_phi;

			sampleDirection=ROTATE_VEC(sampleDirection, startDirection, rotate_phi, phiAxis);

			if( USE_OCTREE )
			{
				phiResamplings.push_back( intersectionPoint( Ray(endPoint,sampleDirection), octree ) );
			}
			else
			{
				// test for all faces
				for(int i=0;i<(int)mesh_faces.size();i++)
				{
					if(rayIntersectsTriangle(endPoint,sampleDirection, mesh_faces[i],intersect_point,radius))
					{
						phiResamplings.push_back(intersect_point);
						break;
					}
				}
			}
		}

		sphereResamplings.push_back(phiResamplings);
	}

	return sphereResamplings;
}

//Modified by http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/ which is based on Moller and Trumbore 1997
//point(u,v) = (1-u-v)*p0 + u*p1 + v*p2	where
//	p0,p1,p2 are the vertices of the triangle 
//	u >= 0
//	v >= 0
//	u + v <= 1.0

//We also know that the parametric equation of the line is:
//	point(t) = p + t * d	where 
//	p is a point in the line
//	d is a vector that provides the line's direction

bool Morpher::rayIntersectsTriangle(Vector3 origin, Vector3 direction, std::vector<Vector3> triangle, Vector3 &intesect_point, double &radius)
{
	Vector3 e1,e2,h,s,q;
	float a,f,u,v,t;
	Vector3 v0=triangle[0];
	Vector3 v1=triangle[1];
	Vector3 v2=triangle[2];
	VEC_FROM_POINTS(e1,v1,v0);
	VEC_FROM_POINTS(e2,v2,v0);


	//crossProduct(h,direction,e2);
	h=cross(direction,e2);
	a = dot(e1,h);

	if (a > -0.00001 && a < 0.00001)
		return(false);

	f = 1/a;
	VEC_FROM_POINTS(s,origin,v0);
	u = f * (dot(s,h));

	if (u < 0.0 || u > 1.0)
		return(false);

	q=cross(s,e1);
	v = f * dot(direction,q);

	if (v < 0.0 || u + v > 1.0)
		return(false);

	// at this stage we can compute t to find out where
	// the intersection point is on the line
	t = f *dot(e2,q);

	if (t > 0.01) // ray intersection
	{
		intesect_point[0]=origin[0]+t*direction[0];
		intesect_point[1]=origin[1]+t*direction[1];
		intesect_point[2]=origin[2]+t*direction[2];

		//		intesect_point[0]=origin[0]+(1-u-v)*v0[0]+u*v1[0]+v*v2[0];
		//		intesect_point[1]=origin[1]+(1-u-v)*v0[1]+u*v1[1]+v*v2[1];
		//		intesect_point[2]=origin[2]+(1-u-v)*v0[2]+u*v1[2]+v*v2[2];

		radius=t*dot(direction,direction);
		return(true);
	}
	else // this means that there is a line intersection
		// but not a ray intersection
		return (false);

}

void Morpher::buildFaces(SurfaceMeshModel *mesh)
{
	foreach(std::vector<SurfaceMeshModel::Vertex> face, facesToBuild)
	{
		mesh->add_face(face);
	}
}

// add faces to SurfaceMesh

void Morpher::addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int vNum=resampledPlane[0].size();
	int uNum=resampledPlane[0][0].size();

	for (int k=0;k<2;k++)
	{
		for(int curr_v=0; curr_v<vNum;curr_v++)
		{
			for(int curr_u=0; curr_u<uNum; curr_u++)
				vertices_idx.push_back(mesh->add_vertex(resampledPlane[k][curr_v][curr_u]));
		}
	}

	// add upside faces
	for (int curr_v= 0; curr_v<vNum-1;curr_v++)
	{
		for(int curr_u=0; curr_u<uNum-1; curr_u++)
		{
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+idxBase]);
			face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+idxBase]);

			mesh->add_face(face_vertex_idx);
		}
	}

	// add downside faces
	for (int curr_v= 0; curr_v<vNum-1;curr_v++)
	{
		for(int curr_u=0; curr_u<uNum-1; curr_u++)
		{
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+idxBase+uNum*vNum]);
			face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+idxBase+uNum*vNum]);
			face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1+idxBase+uNum*vNum]);
			face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1+idxBase+uNum*vNum]);

			mesh->add_face(face_vertex_idx);
		}
	}
}

void Morpher::addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int tNum=crossSecssions.size();
	int thetaNum=crossSecssions[0].size();

	for (int curr_t=0; curr_t< tNum; curr_t++)
	{
		for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
		{
			vertices_idx.push_back(mesh->add_vertex(crossSecssions[curr_t][curr_theta]));
		}
	}

	for (int curr_t= 0; curr_t<tNum-1;curr_t++)
	{
		for(int curr_theta=0; curr_theta<thetaNum-1; curr_theta++)
		{
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[curr_t*thetaNum+curr_theta+idxBase]);
			face_vertex_idx.push_back(vertices_idx[curr_t*thetaNum+curr_theta+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_t+1)*thetaNum+curr_theta+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_t+1)*thetaNum+curr_theta+idxBase]);

			mesh->add_face(face_vertex_idx);
		}
	}
}

void Morpher::addCornerFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int thetaNum=sphereResamplings.size();
	int phiNum=sphereResamplings[0].size();

	for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
	{
		for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
		{
			vertices_idx.push_back(mesh->add_vertex(sphereResamplings[curr_theta][curr_phi]));
		}
	}

	for (int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
	{
		for(int curr_phi=0; curr_phi<phiNum-1; curr_phi++)
		{
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+1+idxBase]);

			mesh->add_face(face_vertex_idx);
		}
	}
}


// theta = longitude, phi = latitude
void Morpher::addEndFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int thetaNum=sphereResamplings.size();
	int phiNum=sphereResamplings[0].size();

	for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
	{
		for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
		{
			vertices_idx.push_back(mesh->add_vertex(sphereResamplings[curr_theta][curr_phi]));
		}
	}

	for (int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
	{
		for(int curr_phi=0; curr_phi<phiNum-1; curr_phi++)
		{
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+idxBase]);
			face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+1+idxBase]);
			face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+1+idxBase]);

			mesh->add_face(face_vertex_idx);
		}
	}

	for(int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+phiNum-1+idxBase]);
		face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+phiNum-1+idxBase]);
		face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+idxBase]);
		face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+idxBase]);

		mesh->add_face(face_vertex_idx);
	}
}

void Morpher::stitchCylinder( int timeResolution, int thetaResolution, int sampling_phiResolution )
{
	// connect the end with the trunk
	int numTrunk= (timeResolution-1)*thetaResolution;
	int numEnd=thetaResolution*(sampling_phiResolution+1);
	int currentTotal=source_verticesIdx.size();

	std::vector<Vertex> face_vertex_idx;
	// t=0, the rotation of sphere sampling is inverse to the trunk
	// curve: startDirection=biNormal
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk+ thetaResolution-1]);

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	for (int curr_theta=1; curr_theta< thetaResolution-1; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;

		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk+thetaResolution-curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk+thetaResolution-curr_theta-1]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+thetaResolution-1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk+1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-numTrunk]);

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	//t=1
	for (int curr_theta=0; curr_theta< thetaResolution-1; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-thetaResolution+curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-thetaResolution+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+curr_theta]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-thetaResolution+thetaResolution-1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numEnd-thetaResolution]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+thetaResolution-1]);

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);
}

void Morpher::stitchPlane( int uResolution, int vResolution, int sampling_thetaResolution, int sampling_phiResolution)
{
	int numCorner=(sampling_thetaResolution+1)*(sampling_phiResolution+1);
	int numTrunckU=(uResolution-1)*(sampling_thetaResolution+1);
	int numTrunckV=(vResolution-1)*(sampling_thetaResolution+1);
	int numOnePlane=(uResolution-1)*(vResolution-1);
	int currentTotal=source_verticesIdx.size();	
	// connect plane to boundary cylinder
	// curve1: v=0
	for (int curr_u=0; curr_u< uResolution-2; curr_u++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_u]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_u*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_u+1]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	for (int curr_u=0; curr_u< uResolution-2; curr_u++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_u]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_u+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_u*(sampling_thetaResolution+1)+sampling_thetaResolution]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// curve2: u=uResolution
	for (int curr_v=0; curr_v< vResolution-2; curr_v++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_v*(uResolution-1)+uResolution-2]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_v*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(curr_v+1)*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(curr_v+1)*(uResolution-1)+uResolution-2]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	for (int curr_v=0; curr_v< vResolution-2; curr_v++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_v*(uResolution-1)+uResolution-2]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(curr_v+1)*(uResolution-1)+uResolution-2]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(curr_v+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_v*(sampling_thetaResolution+1)+sampling_thetaResolution]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// curve3: v=vResolution
	for (int curr_u=0; curr_u< uResolution-2; curr_u++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(uResolution-1)*(vResolution-2)+curr_u]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(uResolution-1)*(vResolution-2)+curr_u+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u)*(sampling_thetaResolution+1)+sampling_thetaResolution]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	for (int curr_u=0; curr_u< uResolution-2; curr_u++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(uResolution-1)*(vResolution-2)+curr_u]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+curr_u*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(uResolution-1)*(vResolution-2)+curr_u+1]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// curve 4 u=0;
	for (int curr_v=0; curr_v< vResolution-2; curr_v++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_v*(uResolution-1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(curr_v+1)*(uResolution-1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+(curr_v+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+curr_v*(sampling_thetaResolution+1)+sampling_thetaResolution]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	for (int curr_v=0; curr_v< vResolution-2; curr_v++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_v*(uResolution-1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+curr_v*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+(curr_v+1)*(sampling_thetaResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(curr_v+1)*(uResolution-1)]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}


	// connect corner to boundary cylinder
	// corner1: u=0, v=0
	// corner1:samplingStartDirection=sheetNormal; curve4: samplingStartDirection=-sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner+curr_theta*(sampling_phiResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution-curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution-curr_theta-1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner1:samplingStartDirection=sheetNormal; curve1: samplingStartDirection=sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_theta]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner2: u=1, v=0
	// corner2:samplingStartDirection=sheetNormal; curve1: samplingStartDirection=sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(uResolution-2)*(sampling_thetaResolution+1)+curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(uResolution-2)*(sampling_thetaResolution+1)+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner+curr_theta*(sampling_phiResolution+1)]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner2:samplingStartDirection=sheetNormal; curve2: samplingStartDirection=sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_theta]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner3: u=1, v=1
	// corner3:samplingStartDirection=sheetNormal; curve2: samplingStartDirection=sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(vResolution-2)*(sampling_thetaResolution+1)+curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(vResolution-2)*(sampling_thetaResolution+1)+curr_theta+1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner+curr_theta*(sampling_phiResolution+1)]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner3:samplingStartDirection=sheetNormal; curve3: samplingStartDirection=-sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU
			+(uResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta-1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU
			+(uResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner4: u=0, v=1
	// corner4:samplingStartDirection=sheetNormal; curve3: samplingStartDirection=-sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;

		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+sampling_thetaResolution-curr_theta]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+sampling_thetaResolution-curr_theta-1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner+curr_theta*(sampling_phiResolution+1)]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// corner4:samplingStartDirection=sheetNormal; curve4: samplingStartDirection=-sheetNormal;
	for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV
			+(vResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta-1]);
		face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV
			+(vResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta]);

		resampledSourceMesh->add_face(face_vertex_idx);
		resampledTargetMesh->add_face(face_vertex_idx);
	}

	// connect plane to corners, 8 faces in total
	// corner1
	std::vector<Vertex> face_vertex_idx;
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution]); // on curve4
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner]); // on corner1
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV]); // on curve1

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV+sampling_thetaResolution]); // on curve1
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner-1]); // on corner1
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV]); // on curve4

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	// corner2
	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane+uResolution-2]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV+(uResolution-2)*(sampling_thetaResolution+1)]); // on curve1
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-3*numCorner]); // on corner2
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV]); // on curve2

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane+uResolution-2]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV+sampling_thetaResolution]); // on curve2
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner-1]); // on corner2
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV-1]); // on curve1

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	// corner3
	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane-1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV+(vResolution-2)*(sampling_thetaResolution+1)]); // on curve2
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-2*numCorner]); // on corner3
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV-1]); // on curve3

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-1]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV+(uResolution-2)*(sampling_thetaResolution+1)]); // on curve3
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner-1]); // on corner3
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV-1]); // on curve2

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	// corner4
	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane+(vResolution-2)*(uResolution-1)]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV+sampling_thetaResolution]); // on curve3
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-numCorner]); // on corner4
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-1]); // on curve4

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

	face_vertex_idx.clear();
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane+(vResolution-2)*(uResolution-1)]);
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckV+(vResolution-2)*(sampling_thetaResolution+1)]); // on curve4
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-1]); // on corner4
	face_vertex_idx.push_back(source_verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV]); // on curve3

	resampledSourceMesh->add_face(face_vertex_idx);
	resampledTargetMesh->add_face(face_vertex_idx);

}

Vec3d Morpher::intersectionPoint( Ray ray, Octree * useTree, int * faceIndex )
{
	HitResult res;
	Vec3d isetpoint(0);

	QSet<int> results = useTree->intersectRay( ray, ray.thickness, false );

	double minDistance = DBL_MAX;
	bool foundIntersection;

	foreach(int i, results)
	{
		useTree->intersectionTestOld(SurfaceMeshModel::Face(i), ray, res);

		//if(res.hit)
		//{
		//	isetpoint = ray.origin + (ray.direction * res.distance);
		//	if(faceIndex) *faceIndex = i;
		//	return isetpoint;
		//}

		// find the nearest intersection point
		if(res.hit)
		{
			if (res.distance < minDistance)
			{
				minDistance = res.distance;
				isetpoint = ray.origin + (ray.direction * res.distance);
				if(faceIndex) *faceIndex = i;
			}
			foundIntersection = true;
		}
	}

	assert(foundIntersection == true);

	return isetpoint;
}

void Morpher::testCase()
{
	std::vector<SurfaceMeshModel*> models;
	std::vector<Structure::Node *> nodes;
	std::vector<Octree*> octrees;

	models.push_back(source_mesh);
	models.push_back(target_mesh);

	nodes.push_back(source_graph.nodes.front());
	nodes.push_back(target_graph.nodes.front());

	octrees.push_back(source_octree);
	octrees.push_back(target_octree);

	double res = source_graph.bbox().size().length() * 0.01;

	// Rays to shoot
	std::vector<Vec4d> rayCoords;
	std::vector<Vec3d> rayDir;

	for(int i = 0; i<(int) models.size();i++)
	{
		SurfaceMeshModel * model = models[i];
		Structure::Node * node = nodes[i];

		// For source
		Array2D_Vec4d sheetPoints = node->discretizedPoints(res);

		// Add all (u,v) coordinates to kd-tree
		NanoKdTree tree;
		Array1D_Vec4d allCoords;
		foreach(Array1D_Vec4d coords, sheetPoints)
		{
			foreach(Vec4d c, coords)
			{
				tree.addPoint( node->position(c) );
				allCoords.push_back(c);
			}
		}
		tree.build();

		// Get closest from mesh
		Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");

		foreach(Vertex v, model->vertices())
		{
			Vec3d meshPoint = points[v];

			KDResults match;
			tree.k_closest(meshPoint, 1, match);
			int closest_idx = match.front().first;
			Vec4d c = allCoords[closest_idx];

			Vec3d sheetPoint = node->position(c);
			Vec3d direction = (meshPoint - sheetPoint).normalized();

			rayCoords.push_back(c);
			rayDir.push_back(direction);
		}
	}

	for(int i = 0; i < (int) models.size();i++)
	{
		// Save to a file
		QFile file(QString("pointcloud_%1.xyz").arg(i));
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
		QTextStream out(&file);
		models[i]->update_face_normals();
		Vector3FaceProperty fnormals = models[i]->face_property<Vec3d>("f:normal");

		for(int r = 0; r < (int) rayCoords.size();r++)
		{
			Ray ray( nodes[i]->position(rayCoords[r]), rayDir[r] );
			int faceIndex = -1;

			Vec3d isect = intersectionPoint(ray, octrees[i], &faceIndex);
			Vec3d vnormal = fnormals[SurfaceMeshModel::Face(faceIndex)];

			// Save position and normal
			out << QString("%1 %2 %3 ").arg(isect[0]).arg(isect[1]).arg(isect[2]) << QString("%1 %2 %3").arg(vnormal[0]).arg(vnormal[1]).arg(vnormal[2]) << "\n";
		}

		file.close();
	}
}

SurfaceMeshModel* Morpher::generateInBetween( std::vector<SurfaceMeshModel*> source_models, std::vector<SurfaceMeshModel*> target_models, std::vector<double> T )
{
	SurfaceMeshModel* blendModel = new SurfaceMeshModel("blend.off","blend");
	int vertexIndexBase = 0;
	std::vector<Vertex> vertiexIndices;
	std::vector<Vector3> finalPoints;

	for (int i=0; i< (int)source_models.size(); i++)
	{
		Vector3VertexProperty source_points = source_models[i]->vertex_property<Vector3>(VPOINT);
		Vector3VertexProperty target_points = target_models[i]->vertex_property<Vector3>(VPOINT);

		Surface_mesh::Vertex_iterator vit, vend=source_models[i]->vertices_end();
	}

	for (int i=0; i< (int)source_models.size(); i++)
	{
		Vector3VertexProperty source_points = source_models[i]->vertex_property<Vector3>(VPOINT);
		Vector3VertexProperty target_points = target_models[i]->vertex_property<Vector3>(VPOINT);

		Surface_mesh::Vertex_iterator vit, vend=source_models[i]->vertices_end();

		for (vit=source_models[i]->vertices_begin(); vit!=vend; ++vit)
		{
			Vector3 p = (1-T[i])*source_points[vit]+T[i]*target_points[vit];
			vertiexIndices.push_back( blendModel->add_vertex(p) );
		}

		// Go over faces
		foreach(Face f, source_models[i]->faces())
		{
			std::vector<Vertex> face_vertex_idx;
			std::vector<Vertex> Face_vertex_idx;
			Surface_mesh::Vertex_around_face_circulator vfit = source_models[i]->vertices(f);

			face_vertex_idx.push_back(vfit);
			face_vertex_idx.push_back(++vfit);
			face_vertex_idx.push_back(++vfit);
			face_vertex_idx.push_back(++vfit);

			//since we read multiple models, the vertex index should be added by a base index number of previous saved vertex number 
			for(int v=0; v < (int)face_vertex_idx.size(); v++)
				Face_vertex_idx.push_back( vertiexIndices[face_vertex_idx[v].idx() + vertexIndexBase] );

			blendModel->add_face(Face_vertex_idx);
		}

		vertexIndexBase = blendModel->n_vertices();
	}

	blendModel = mergeVertices(blendModel);

	return blendModel;
}

SurfaceMeshModel* Morpher::mergeVertices( SurfaceMeshModel * model, double threshold )
{
	std::vector<Vec3d> partPoints;

	Vector3VertexProperty points = model->vertex_property<Vector3>(VPOINT);
	Surface_mesh::Vertex_iterator vit, vend=model->vertices_end();

	for (vit=model->vertices_begin(); vit!=vend; ++vit)
	{
		Vector3 p = points[vit];

		foreach(Vec3d q, partPoints){
			if( (q - p).norm() < threshold){
				p = q;
				break;
			}
		}

		partPoints.push_back( p );
	}

	// Merge duplicate vertices
	SurfaceMeshModel * mergedMesh = new SurfaceMeshModel("merged.off","Merged");
	std::vector<size_t> xrefs;
	weld(partPoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());

    for(int vi = 0; vi < (int)partPoints.size(); vi++)
		mergedMesh->add_vertex(partPoints[vi]);

	foreach(Face f, model->faces())
	{
		std::vector<Vertex> verts;
		Surface_mesh::Vertex_around_face_circulator vit = model->vertices(f),vend=vit;
		QVector<int> vset;
		do{
			int newIdx = xrefs[Vertex(vit).idx()];
			if(!vset.contains(newIdx)) vset.push_back(newIdx);
		} while(++vit != vend);

		foreach(int vidx, vset)	verts.push_back(Vertex(vidx));

		mergedMesh->add_face(verts);
	}

	return mergedMesh;
}

