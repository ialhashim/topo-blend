#include "Morpher.h"
#include "surface_mesh/IO.h"
#include "surface_mesh/IO_off.cpp"

#define SIGN(x) (((x) < 0) ? (-1) : (((x) > 0) ? 1 : 0))
#define ROTATE_VEC(u, v, theta, axis) (u = v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)))
#define VEC_FROM_POINTS(a,b,c) \
	(a)[0] = (b)[0] - (c)[0];	\
	(a)[1] = (b)[1] - (c)[1];	\
	(a)[2] = (b)[2] - (c)[2];

Morpher::Morpher(SurfaceMeshModel *mesh1, SurfaceMeshModel *mesh2, Graph graph1, Graph graph2,	QObject *parent)
	: QObject(parent)
{
	source_mesh=mesh1;
	target_mesh=mesh2;

	source_graph=graph1;
	target_graph=graph2;

	get_faces(source_mesh, target_mesh, source_faces, target_faces);
	buildOctree(source_mesh,target_mesh,source_octree,target_octree,50);
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
	std::vector<Surface_mesh::Face> allTris1, allTris2;
	foreach(Face f, source_mesh->faces()) allTris1.push_back(f);
	foreach(Face f, target_mesh->faces()) allTris2.push_back(f);

	source_octree = new Octree(numTrisPerNode, source_mesh);
	target_octree=new Octree(numTrisPerNode, target_mesh);

	source_octree->initBuild(allTris1, numTrisPerNode);
	target_octree->initBuild(allTris2, numTrisPerNode);
}

void Morpher::generateInBetween(int numStep, int uResolution, int vResolution, int timeResolution, int thetaResolution, int phiResolution)
{
	SurfaceMeshModel resampledSourceMesh;
	SurfaceMeshModel resampledTargetMesh;
	std::vector<Vertex> vertices_idx;
	std::vector<Vertex> source_verticesIdx, target_verticesIdx;
	
	Array2D_Vector3 souce_sampleDirections, target_sampleDirections;
	std::vector<std::vector<double>> souce_radii, target_radii;
	NURBSCurve source_curve, target_curve;
	NURBSRectangle source_sheet, target_sheet;


	//transverse the graph nodes
	//do cross section resampling to curve nodes
	for(int i=0;i<source_graph.nodes.size();i++)
	{
		if(source_graph.nodes[i]->type()==Structure::CURVE&&target_graph.nodes[i]->type()==Structure::CURVE)
		{

			Curve* sourceCurve=(Curve *)source_graph.nodes[i];
			Curve* targetCurve=(Curve *)target_graph.nodes[i];
			source_curve=sourceCurve->curve;
			target_curve=targetCurve->curve;

			Vector3 initialSourceDirection, initialTargetDirection;
			double thetaRange=2*M_PI;
			double phiRange=M_PI/2;

			// set the initial theta sampling direction as the curve normal
			initialSourceDirection=source_curve.GetBinormal(0);
			initialTargetDirection=target_curve.GetBinormal(0);

			// align initial sampling direction
			if(dot(initialSourceDirection,initialTargetDirection)<0)
				initialTargetDirection=-1*initialTargetDirection;

			Array2D_Vector3 source_crossSections=cylinderResampling(source_faces,source_curve,initialSourceDirection,timeResolution,(int) thetaResolution*thetaRange/(2*M_PI),thetaRange, source_octree, false);
			Array2D_Vector3 target_crossSections=cylinderResampling(target_faces,target_curve,initialTargetDirection,timeResolution,(int)thetaResolution*thetaRange/(2*M_PI),thetaRange, target_octree, false);

			addCylinderFaces(source_crossSections, resampledSourceMesh, source_verticesIdx, 0);
			addCylinderFaces(target_crossSections, resampledTargetMesh, target_verticesIdx, 0);

			int tNum=source_crossSections.size();
			int thetaNum=source_crossSections[0].size();
			int idxBase=0;
			for(int curr_t=0; curr_t<tNum-1;curr_t++)
			{
				std::vector<Vertex> face_vertex_idx;
				face_vertex_idx.push_back(source_verticesIdx[curr_t*thetaNum+thetaNum-1+idxBase]);
				face_vertex_idx.push_back(source_verticesIdx[curr_t*thetaNum+idxBase]);
				face_vertex_idx.push_back(source_verticesIdx[(curr_t+1)*thetaNum+idxBase]);
				face_vertex_idx.push_back(source_verticesIdx[(curr_t+1)*thetaNum+thetaNum-1+idxBase]);

				resampledSourceMesh.add_face(face_vertex_idx);
				resampledTargetMesh.add_face(face_vertex_idx);
			}

			Vector3 source_pos, sourceTangent, sourceBinormal, sourceNormal;
			Vector3 target_pos, targetTangent, targetBinormal, targetNormal;

			// t=0
			source_pos=source_curve.GetPosition(0);
			sourceTangent=source_curve.GetTangent(0);
			sourceNormal=source_curve.GetNormal(0);

			target_pos=target_curve.GetPosition(0);
			targetTangent=target_curve.GetTangent(0);
			targetNormal=target_curve.GetNormal(0);

			assert(dot(sourceTangent,targetTangent)>0.5);

			if (dot(sourceNormal,targetNormal)<0)
			{
				targetNormal=-targetNormal;
			}

			// here, the meaning of theta changes:
			// the theta in the cylinder represents the phi in the sphere
			thetaRange=2*M_PI;
			phiRange=M_PI/2;
			
			Array2D_Vector3 source_endResamplings1=sphereResampling(source_faces,source_pos, sourceNormal, -sourceTangent,
				(int)phiResolution*phiRange/(M_PI), (int)thetaResolution*thetaRange/(2*M_PI), phiRange, thetaRange, source_octree);
			Array2D_Vector3 target_endResamplings1=sphereResampling(target_faces,target_pos, targetNormal, -targetTangent,
				(int)phiResolution*phiRange/(M_PI), (int)thetaResolution*thetaRange/(2*M_PI), phiRange, thetaRange, target_octree);

			addEndFaces(source_endResamplings1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addEndFaces(target_endResamplings1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// t=1
			source_pos=source_curve.GetPosition(1);
			sourceTangent=source_curve.GetTangent(1);
			sourceNormal=source_curve.GetNormal(1);

			target_pos=target_curve.GetPosition(1);
			targetTangent=target_curve.GetTangent(1);
			targetNormal=target_curve.GetNormal(1);

			assert(dot(sourceTangent,targetTangent)>0.5);

			if (dot(sourceNormal,targetNormal)<0)
			{
				targetNormal=-targetNormal;
			}

			Array2D_Vector3 source_endResamplings2=sphereResampling(source_faces,source_pos, sourceNormal, sourceTangent,
				(int)phiResolution*phiRange/(M_PI), (int)thetaResolution*thetaRange/(2*M_PI), phiRange, thetaRange, source_octree);
			Array2D_Vector3 target_endResamplings2=sphereResampling(target_faces,target_pos, targetNormal, targetTangent,
				(int)phiResolution*phiRange/(M_PI), (int)thetaResolution*thetaRange/(2*M_PI), phiRange, thetaRange, target_octree);

			addEndFaces(source_endResamplings2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addEndFaces(target_endResamplings2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());
			

			resampledSourceMesh.write("resampled_source.off");
			resampledTargetMesh.write("resampled_target.off");

			linear_Interp_Corrd2(resampledSourceMesh,resampledTargetMesh, numStep);
		}

		if(source_graph.nodes[i]->type()==Structure::SHEET&&target_graph.nodes[i]->type()==Structure::SHEET)
		{

			Sheet* sourceSheet=(Sheet*) source_graph.nodes[i];
			Sheet* targetSheet=(Sheet*) target_graph.nodes[i];
			source_sheet=sourceSheet->surface;
			target_sheet=targetSheet->surface;

			std::vector<Array2D_Vector3> resampledSoucePlane=planeResamping(source_faces,source_sheet,uResolution,vResolution, source_octree);
			std::vector<Array2D_Vector3> resampledTargetPlane=planeResamping(target_faces,target_sheet,uResolution,vResolution, target_octree);


			addPlaneFaces(resampledSoucePlane, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addPlaneFaces(resampledTargetPlane, resampledTargetMesh,target_verticesIdx, target_verticesIdx.size());

			// sampling using the sheet boundaries, counter-clockwise
			// curve1: transverse u while v=0
			std::vector<Vector3> source_ctrlPoints1=source_sheet.GetControlPointsU(0);
			std::vector<Real> source_ctrlWeights1(source_ctrlPoints1.size(), 1.0);
			std::vector<Vector3> target_ctrlPoints1=target_sheet.GetControlPointsU(0);
			std::vector<Real> target_ctrlWeights1(target_ctrlPoints1.size(), 1.0);
			NURBSCurve source_curve1=NURBSCurve(source_ctrlPoints1, source_ctrlWeights1, 3, false, true);
			NURBSCurve target_curve1=NURBSCurve(target_ctrlPoints1, target_ctrlWeights1, 3, false, true);

			Vector3 initialSourceDirection, initialTargetDirection;
			double thetaRange=M_PI;
			double phiRange=M_PI/2;

			Vector3 position, sheetNormal, tangentU, tangentV;
			// set the initial theta sampling direction as the curve normal
            source_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
			initialSourceDirection=sheetNormal;

			target_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
			initialTargetDirection=sheetNormal;
			assert(dot(initialSourceDirection,initialTargetDirection)>0.5);


			Array2D_Vector3 source_crossSections1=cylinderResampling(source_faces,source_curve1,initialSourceDirection,
				uResolution,(int)thetaResolution*thetaRange/(2*M_PI),thetaRange, source_octree, true);
			Array2D_Vector3 target_crossSections1=cylinderResampling(target_faces,target_curve1,initialTargetDirection,
				uResolution,(int)thetaResolution*thetaRange/(2*M_PI), thetaRange, target_octree, true);

			addCylinderFaces(source_crossSections1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCylinderFaces(target_crossSections1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// curve2: transverse v while u=uResolution

			std::vector<Vector3> source_ctrlPoints2=source_sheet.GetControlPointsV(source_sheet.mNumVCtrlPoints-1);
			std::vector<Real> source_ctrlWeights2(source_ctrlPoints2.size(), 1.0);
			std::vector<Vector3> target_ctrlPoints2=target_sheet.GetControlPointsV(target_sheet.mNumVCtrlPoints-1);
			std::vector<Real> target_ctrlWeights2(target_ctrlPoints2.size(), 1.0);
			NURBSCurve source_curve2=NURBSCurve(source_ctrlPoints2, source_ctrlWeights2, 3, false, true);
			NURBSCurve target_curve2=NURBSCurve(target_ctrlPoints2, target_ctrlWeights2, 3, false, true);

			Array2D_Vector3 source_crossSections2=cylinderResampling(source_faces,source_curve2,initialSourceDirection,
				vResolution,(int)thetaResolution*thetaRange/(2*M_PI),thetaRange, source_octree, true);
			Array2D_Vector3 target_crossSections2=cylinderResampling(target_faces,target_curve2,initialTargetDirection,
				vResolution,(int)thetaResolution*thetaRange/(2*M_PI), thetaRange, target_octree, true);

			addCylinderFaces(source_crossSections2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCylinderFaces(target_crossSections2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// curve3: transverse u while v=vResolution
			std::vector<Vector3> source_ctrlPoints3=source_sheet.GetControlPointsU(source_sheet.mNumUCtrlPoints-1);
			std::vector<Real> source_ctrlWeights3(source_ctrlPoints3.size(), 1.0);
			std::vector<Vector3> target_ctrlPoints3=target_sheet.GetControlPointsU(source_sheet.mNumUCtrlPoints-1);
			std::vector<Real> target_ctrlWeights3(target_ctrlPoints3.size(), 1.0);
			NURBSCurve source_curve3=NURBSCurve(source_ctrlPoints3, source_ctrlWeights3, 3, false, true);
			NURBSCurve target_curve3=NURBSCurve(target_ctrlPoints3, target_ctrlWeights3, 3, false, true);

			initialSourceDirection=-sheetNormal;
			initialTargetDirection=-sheetNormal;
			Array2D_Vector3 source_crossSections3=cylinderResampling(source_faces,source_curve3,initialSourceDirection,
				uResolution,(int)thetaResolution*thetaRange/(2*M_PI),thetaRange, source_octree,true);
			Array2D_Vector3 target_crossSections3=cylinderResampling(target_faces,target_curve3,initialTargetDirection,
				uResolution,(int)thetaResolution*thetaRange/(2*M_PI), thetaRange,target_octree,true);

			addCylinderFaces(source_crossSections3, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCylinderFaces(target_crossSections3, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// curve4: transverse v while u=0
			std::vector<Vector3> source_ctrlPoints4=source_sheet.GetControlPointsV();
			std::vector<Real> source_ctrlWeights4(source_ctrlPoints4.size(), 1.0);
			std::vector<Vector3> target_ctrlPoints4=target_sheet.GetControlPointsV();
			std::vector<Real> target_ctrlWeights4(target_ctrlPoints4.size(), 1.0);
			NURBSCurve source_curve4=NURBSCurve(source_ctrlPoints4, source_ctrlWeights4, 3, false, true);
			NURBSCurve target_curve4=NURBSCurve(target_ctrlPoints4, target_ctrlWeights4, 3, false, true);

			initialSourceDirection=-sheetNormal;
			initialTargetDirection=-sheetNormal;
			Array2D_Vector3 source_crossSections4=cylinderResampling(source_faces,source_curve4,initialSourceDirection,
				vResolution,(int)thetaResolution*thetaRange/(2*M_PI),thetaRange,source_octree,true);
			Array2D_Vector3 target_crossSections4=cylinderResampling(target_faces,target_curve4,initialTargetDirection,
				vResolution,(int)thetaResolution*thetaRange/(2*M_PI), thetaRange,target_octree,true);

			addCylinderFaces(source_crossSections4, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCylinderFaces(target_crossSections4, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// corner1: u=0, v=0
			source_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 source_cornerResamplings1=sphereResampling(source_faces,position,-tangentV,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, source_octree);

			target_sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 target_cornerResamplings1=sphereResampling(target_faces,position,-tangentV,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, target_octree);

			addCornerFaces(source_cornerResamplings1, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCornerFaces(target_cornerResamplings1, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// corner2: u=1, v=0
			source_sheet.GetFrame(1,0,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 source_cornerResamplings2=sphereResampling(source_faces,position,tangentU,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, source_octree);
			
			target_sheet.GetFrame(1,0,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 target_cornerResamplings2=sphereResampling(target_faces,position,tangentU,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, target_octree);

			addCornerFaces(source_cornerResamplings2, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCornerFaces(target_cornerResamplings2, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// corner3: u=1, v=1
			source_sheet.GetFrame(1,1,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 source_cornerResamplings3=sphereResampling(source_faces,position,tangentV,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, source_octree);

			target_sheet.GetFrame(1,1,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 target_cornerResamplings3=sphereResampling(target_faces,position,tangentV,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, target_octree);

			addCornerFaces(source_cornerResamplings3, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCornerFaces(target_cornerResamplings3, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());

			// corner4: u=0, v=1
			source_sheet.GetFrame(0,1,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 source_cornerResamplings4=sphereResampling(source_faces,position,-tangentU,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, source_octree);

			target_sheet.GetFrame(0,1,position, tangentU, tangentV, sheetNormal);
			Array2D_Vector3 target_cornerResamplings4=sphereResampling(target_faces,position,-tangentU,sheetNormal,
				(int)thetaResolution*thetaRange/(2*M_PI),(int)phiResolution*phiRange/(2*M_PI), thetaRange, phiRange, target_octree);

			addCornerFaces(source_cornerResamplings4, resampledSourceMesh, source_verticesIdx, source_verticesIdx.size());
			addCornerFaces(target_cornerResamplings4, resampledTargetMesh, target_verticesIdx, target_verticesIdx.size());
	
			resampledSourceMesh.write("resampled_source.off");
			resampledTargetMesh.write("resampled_target.off");

			linear_Interp_Corrd2(resampledSourceMesh,resampledTargetMesh, numStep);
		}
	}
}

void Morpher::linear_Interp_Corrd2(SurfaceMeshModel &source, SurfaceMeshModel &target, int numStep)
{
	Vector3VertexProperty source_points = source.vertex_property<Vector3>(VPOINT);
	Vector3VertexProperty target_points = target.vertex_property<Vector3>(VPOINT);

	for(int step=0; step<=numStep;step++)
	{
		SurfaceMeshModel blend;
		double blend_time=step*1.0/numStep;
		foreach(Vertex v, source.vertices())
			blend.add_vertex((1-blend_time)*source_points[v]+blend_time*target_points[v]);

		foreach(Face f, source.faces() )
		{
			 Surface_mesh::Vertex_around_face_circulator vfit = source.vertices(f);
			 std::vector<Vertex> face_vertices;
			 face_vertices.push_back(vfit);
			 face_vertices.push_back(++vfit);
			 face_vertices.push_back(++vfit);
			 face_vertices.push_back(++vfit);

			 blend.add_face(face_vertices);
		}

		blend.triangulate();
		QString seq_num; seq_num.sprintf("%02d", step);
		blend.write(QString("blend%1.off").arg(seq_num).toStdString());
	}
	
}

// for a true cylinder (close, thetaSamplings=theResolution), and sampling direction computed as the binormal each time
// status: octree work
// current: using octree
std::vector<Vector3> Morpher::resampleCoutourPoints(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
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
/*
		for(int i=0;i<(int)mesh_faces.size();i++)
		{
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
			{
				intersections.push_back(intersect_point);
				break;
			}
		}
		*/
		// using octree
		IndexSet results = octree->intersectRay( Ray( current_position, sampleDirection ) );
		foreach(int i, results)
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
			{
				intersections.push_back(intersect_point);
				break;
			}
	}

	return intersections;
}

// cylinder sampling for plane (open, thetaSamplings=theResolution+1), and sampling direction fixed as the sheet normal
// status: only all faces work
// current: using all faces
std::vector<Vector3> Morpher::resampleCoutourPoints3(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
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

		///////-------------------/////////////////
		// test for all faces
		for(int i=0;i<(int)mesh_faces.size();i++)
		{
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
			{
				intersections.push_back(intersect_point);
				break;
			}
		}
		///////-------------------/////////////////
		
		// using octree
		///////-------------------/////////////////
	/*	
		IndexSet results = octree->intersectRay( Ray( current_position, sampleDirection ) );
		foreach(int i, results)
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
			{
				intersections.push_back(intersect_point);
				break;
			}
		*/
		///////-------------------/////////////////
	}

	return intersections;
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

// add faces to SurfaceMesh

void Morpher::addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int vNum=resampledPlane[0].size();
	int uNum=resampledPlane[0][0].size();

	for (int k=0;k<2;k++)
	{
		for(int curr_v=0; curr_v<vNum;curr_v++)
		{
			for(int curr_u=0; curr_u<uNum; curr_u++)
				vertices_idx.push_back(mesh.add_vertex(resampledPlane[k][curr_v][curr_u]));
		}
	}

		// add upside faces
		for (int curr_v= 0; curr_v<vNum-1;curr_v++)
		{
			for(int curr_u=0; curr_u<uNum-1; curr_u++)
			{
				std::vector<Vertex> face_vertex_idx;
				face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u]);
				face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1]);
				face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1]);
				face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u]);

				mesh.add_face(face_vertex_idx);
			}
		}

		// add downside faces
		for (int curr_v= 0; curr_v<vNum-1;curr_v++)
		{
			for(int curr_u=0; curr_u<uNum-1; curr_u++)
			{
				int idxBase=uNum*vNum;
				std::vector<Vertex> face_vertex_idx;
				face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+idxBase]);
				face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+idxBase]);
				face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1+idxBase]);
				face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1+idxBase]);

				mesh.add_face(face_vertex_idx);
			}
		}
	
}

void Morpher::addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int tNum=crossSecssions.size();
	int thetaNum=crossSecssions[0].size();

	for (int curr_t=0; curr_t< tNum; curr_t++)
	{
		for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
		{
			vertices_idx.push_back(mesh.add_vertex(crossSecssions[curr_t][curr_theta]));
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

			mesh.add_face(face_vertex_idx);
		}
	}


}

void Morpher::addCornerFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int thetaNum=sphereResamplings.size();
	int phiNum=sphereResamplings[0].size();

	for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
	{
		for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
		{
			vertices_idx.push_back(mesh.add_vertex(sphereResamplings[curr_theta][curr_phi]));
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

			mesh.add_face(face_vertex_idx);
		}
	}
}

void Morpher::addEndFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel &mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
	int thetaNum=sphereResamplings.size();
	int phiNum=sphereResamplings[0].size();

	for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
	{
		for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
		{
			vertices_idx.push_back(mesh.add_vertex(sphereResamplings[curr_theta][curr_phi]));
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

			mesh.add_face(face_vertex_idx);
		}
	}

	for(int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
	{
		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+phiNum-1+idxBase]);
		face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+phiNum-1+idxBase]);
		face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+idxBase]);
		face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+idxBase]);

		mesh.add_face(face_vertex_idx);
	}
}


// status: octree work
// current: using octree
std::vector<Array2D_Vector3> Morpher::planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, int uResolution, int vResolution, Octree* octree)
{
	Array2D_Vector3 upsidePlane, downsidePlane;
	std::vector<Array2D_Vector3> resampledPlane;

	double delta_u=1.0/uResolution, delta_v=1.0/vResolution;

	for(int v_idx=0;v_idx<=vResolution;v_idx++)
	{
		Vector3 sheetPoint, intersectPoint;
		Vector3 uDirection, vDirection, sheetNormal;
		double dist;
		
		Array1D_Vector3 upIntersections, downIntersections;

		for(int u_idx=0;u_idx<=uResolution;u_idx++)
		{

				sheet.GetFrame(u_idx*delta_u, v_idx*delta_v, sheetPoint, uDirection, vDirection, sheetNormal);

				IndexSet results1 = octree->intersectRay( Ray( sheetPoint, sheetNormal ) );
				foreach(int i, results1)
					if(rayIntersectsTriangle(sheetPoint,sheetNormal, mesh_faces[i],intersectPoint,dist))
					{
						upIntersections.push_back(intersectPoint);
						break;
					}

				IndexSet results2 = octree->intersectRay( Ray( sheetPoint, -sheetNormal ) );
				foreach(int i, results2)
					if(rayIntersectsTriangle(sheetPoint,-sheetNormal, mesh_faces[i],intersectPoint,dist))
					{
						downIntersections.push_back(intersectPoint);
						break;
					}				
		}

		upsidePlane.push_back(upIntersections);
		downsidePlane.push_back(downIntersections);
	}
	resampledPlane.push_back(upsidePlane);
	resampledPlane.push_back(downsidePlane);
	
	return resampledPlane;

}

Array2D_Vector3 Morpher::cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection,
	int timeResolution, int thetaResolution, double thetaRange, Octree* octree, bool SheetBoundary)
{
	double delta_t=1.0/timeResolution;

	std::vector<Vector3> cross_section;
	Array2D_Vector3 crossSections;

	for (int i=0;i<=timeResolution;i++)
	{

		// type check
		// cylinder sampling for plane (open, thetaSamplings=theResolution+1), and sampling direction fixed as the sheet normal
		// status: only all faces work now
		if(SheetBoundary)
		cross_section=resampleCoutourPoints3(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);

		// for a true cylinder (close, thetaSamplings=theResolution), and sampling direction computed as the binormal each time
		// status: octree work now
		else
		cross_section=resampleCoutourPoints(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);

		crossSections.push_back(cross_section);
	}

	return crossSections;
}

// phiAxis is set to the sampling start direction, the first intersection is also the pole of the sphere
// first rotate theta around thetaAxis, at a fixed theta, rotate phi around phiAxis
// status: only all faces work 
// current: using all faces
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
			
			// test for all faces
			
			for(int i=0;i<(int)mesh_faces.size();i++)
			{
				if(rayIntersectsTriangle(endPoint,sampleDirection, mesh_faces[i],intersect_point,radius))
				{
					phiResamplings.push_back(intersect_point);
					break;
				}
			}

			// using octree
		///////-------------------/////////////////
			/*
			IndexSet results = octree->intersectRay( Ray( endPoint, sampleDirection ) );
			foreach(int i, results)
				if(rayIntersectsTriangle(endPoint,sampleDirection, mesh_faces[i],intersect_point,radius))
				{
					phiResamplings.push_back(intersect_point);
					break;
				}
				*/
			///////-------------------/////////////////
			
				
		}
		sphereResamplings.push_back(phiResamplings);
	}

	return sphereResamplings;
}