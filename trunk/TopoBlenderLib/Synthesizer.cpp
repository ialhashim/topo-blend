#include <omp.h>

#include <QFile>
#include <QTextStream>

#include "NanoKdTree.h"
#include "Octree.h"
#include "SpherePackSampling.h"
#include "IsotropicRemesher.h"
#include "SimilarSampling.h"
#include "PCA.h"

#include "Synthesizer.h"
#include "weld.h"
Q_DECLARE_METATYPE(RMF)
Q_DECLARE_METATYPE(RMF::Frame)
Q_DECLARE_METATYPE(std::vector<RMF::Frame>)
Q_DECLARE_METATYPE(NanoKdTree)
Q_DECLARE_METATYPE( Structure::Sheet* )

typedef std::pair<ParameterCoord,int> ParameterCoordInt;
bool comparatorParameterCoordInt ( const ParameterCoordInt& l, const ParameterCoordInt& r)
{ return l.first.u < r.first.u; }

// Parameters
#define CURVE_FRAME_RESOLUTION 0.01
#define SHEET_FRAME_RESOLUTION 0.01
#define CURVE_FRAME_COUNT 101	// to match the resolution 0.01

#define OCTREE_NODE_SIZE 40

// Sampling
#define RANDOM_COUNT 1e3
#define UNIFORM_RESOLUTION 0.01f
#define EDGE_RESOLUTION 0.05
#define UNIFORM_TRI_SAMPLES 1e4

// Remeshing
#define REMESH_EDGE_LENGTH 0.005

int global_countt = 0;

int randomCount = RANDOM_COUNT;
int uniformTriCount = UNIFORM_TRI_SAMPLES;

// Helper functions
RMF Synthesizer::consistentFrame( Structure::Curve * curve, Array1D_Vector4d & coords )
{
	// Generate consistent frames along curve
	std::vector<Scalar> times;
	curve->curve.SubdivideByLengthTime(CURVE_FRAME_COUNT, times);
	foreach(Scalar t, times) coords.push_back(Vector4d(t,0,0,0));
	std::vector<Vector3d> samplePoints;
	foreach(Vector4d c, coords) samplePoints.push_back( curve->position(c) );

    RMF rmf = RMF( samplePoints );
	rmf.compute();
	//rmf.generate();

	// Save RMF frames
	curve->property["rmf_frames"].setValue(rmf.U);

	return rmf;
}


/// SAMPLING
// Ray parameters from points
QVector<ParameterCoord> Synthesizer::genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vector3f> & points, const std::vector<Vector3f> & normals )
{
	QVector<ParameterCoord> samples(points.size());

	// Generate consistent frames along curve
	Array1D_Vector4d coords;
	RMF rmf = consistentFrame(curve,coords);

	// Add all curve points to kd-tree
	NanoKdTree kdtree;
	foreach(Vector3d p, rmf.point) kdtree.addPoint( p );
	kdtree.build();

	// Project
	const std::vector<Vector3d> curvePnts = curve->curve.mCtrlPoint;
	int N = points.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSCurved mycurve = NURBS::NURBSCurved::createCurveFromPoints( curvePnts );

		float theta, psi;

		Vector3f point = points[i];

		KDResults match;
		kdtree.k_closest(point.cast<double>(), 1, match);
		int closest_idx = match.front().first;
		Vector4d c = coords[closest_idx];

		Vector3f X = rmf.U[closest_idx].r.normalized().cast<float>();
		Vector3f Y = rmf.U[closest_idx].s.normalized().cast<float>();
		Vector3f Z = rmf.U[closest_idx].t.normalized().cast<float>();

		Vector3f curvePoint = mycurve.GetPosition(c[0]).cast<float>();
		Vector3f delta = point - curvePoint;
		Vector3f raydirection = delta.normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

		samples[i] = ParameterCoord(theta, psi, c[0], 0, delta.norm(), curve);
		samples[i].origNormal = normals[i];
	}

	return samples;
}

QVector<ParameterCoord> Synthesizer::genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vector3f> & points, const std::vector<Vector3f> & normals )
{
	QVector<ParameterCoord> samples(points.size());

	float resolution = sheet->bbox().diagonal().norm() * SHEET_FRAME_RESOLUTION;

	Array2D_Vector4d sheetCoords = sheet->discretizedPoints(resolution);
	Array1D_Vector4d allCoords;

	qDebug() << "Sheet resolution count = " << sheetCoords.size();

	NanoKdTree kdtree;
	foreach(Array1D_Vector4d row, sheetCoords){
		foreach(Vector4d c, row) {
			kdtree.addPoint( sheet->position(c) );
			allCoords.push_back(c);
		}
	}
	kdtree.build();


	int N = points.size();
	const Array2D_Vector3 sheetPnts = sheet->surface.mCtrlPoint;

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSRectangled r = NURBS::NURBSRectangled::createSheetFromPoints(sheetPnts);

		Vector3f point = points[i];

		// Project
		Vector3d X, Y, Z;
		Vector3d vDirection, sheetPoint;

		double theta, psi;

		KDResults match;
		kdtree.k_closest(point.cast<double>(), 1, match);
		int closest_idx = match.front().first;
		Vector4d c = allCoords[closest_idx];

		r.GetFrame( c[0], c[1], sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);
		
		Vector3d delta = point.cast<double>() - sheetPoint;
		Vector3d raydirection = delta.normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

		samples[i] = ParameterCoord(theta, psi, c[0], c[1], delta.norm(), sheet);
		samples[i].origNormal = normals[i];
		samples[i].origPoint = point;
	}

    return samples;
}

// Different sampling methods to generate rays
QVector<ParameterCoord> Synthesizer::genFeatureCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vector3>(VPOINT);
	std::vector<Vector3f> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v].cast<float>());

	// Normals
	model->update_face_normals();
	model->update_vertex_normals();
	Vector3VertexProperty normals = model->vertex_property<Vector3>(VNORMAL);
	std::vector<Vector3f> meshNormals;
	foreach(Vertex v, model->vertices()) meshNormals.push_back(normals[v].cast<float>());

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, meshPoints, meshNormals);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, meshPoints, meshNormals);
}

QVector<ParameterCoord> Synthesizer::genEdgeCoords( Structure::Node * node)
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Sample mesh surface
	QVector<Vector3d> sample_normals;
	QVector<Vector3d> sample_points = SimilarSampler::EdgeUniform( model, uniformTriCount, sample_normals );
	std::vector<Vector3d> samplePoints = sample_points.toStdVector();
	std::vector<Vector3d> sampleNormals = sample_normals.toStdVector();

	// Double to float...
	std::vector<Vector3f> samplePointsF,sampleNormalsF;
	foreach(Vector3d p, sample_points) samplePointsF.push_back(p.cast<float>());
	foreach(Vector3d n, sampleNormals) sampleNormalsF.push_back(n.cast<float>());

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePointsF, sampleNormalsF);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePointsF, sampleNormalsF);
}

QVector<ParameterCoord> Synthesizer::genRandomCoords(Node *node, int samples_count)
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Sample mesh surface
	std::vector<Vector3d> samplePoints, sampleNormals;
	samplePoints = SpherePackSampling::getRandomSamples(model, samples_count, sampleNormals);

	// Double to float...
	std::vector<Vector3f> samplePointsF,sampleNormalsF;
	foreach(Vector3d p, samplePoints) samplePointsF.push_back(p.cast<float>());
	foreach(Vector3d n, sampleNormals) sampleNormalsF.push_back(n.cast<float>());

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePointsF, sampleNormalsF);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePointsF, sampleNormalsF);
}

QVector<ParameterCoord> Synthesizer::genUniformCoords(Node *node, float sampling_resolution)
{
    SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

    // Auto resolution
    if(sampling_resolution < 0) sampling_resolution = UNIFORM_RESOLUTION;

    // Sample mesh surface
    std::vector<Vector3d> samplePoints, sampleNormals, gp;
    samplePoints = SpherePackSampling::sample(model, 1e4, sampling_resolution, gp, sampleNormals, 1);

	// Double to float...
	std::vector<Vector3f> samplePointsF,sampleNormalsF;
	foreach(Vector3d p, samplePoints) samplePointsF.push_back(p.cast<float>());
	foreach(Vector3d n, sampleNormals) sampleNormalsF.push_back(n.cast<float>());

    if(node->type() == Structure::CURVE)
        return genPointCoordsCurve((Structure::Curve*)node, samplePointsF, sampleNormalsF);
    else
        return genPointCoordsSheet((Structure::Sheet*)node, samplePointsF, sampleNormalsF);
}

QVector<ParameterCoord> Synthesizer::genRemeshCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();
	SurfaceMesh::Model copyModel;

	std::vector<Vector3f> samplePoints, sampleNormals;

	Vector3VertexProperty points = model->vertex_property<Vector3d>(VPOINT);

	foreach(Vertex v, model->vertices()) copyModel.add_vertex( points[v] );
	foreach(Face f, model->faces()){
		std::vector<Vertex> verts;
		Surface_mesh::Vertex_around_face_circulator vit = model->vertices(f),vend=vit;
		do{ verts.push_back(vit); } while(++vit != vend);
		copyModel.add_face(verts);
	}

	IsotropicRemesher remesher(&copyModel);
	remesher.doRemesh( REMESH_EDGE_LENGTH );
	
	// Position
	Vector3VertexProperty new_points = copyModel.vertex_property<Vector3d>(VPOINT);
	foreach(Vertex v, copyModel.vertices()) samplePoints.push_back( new_points[v].cast<float>() );
	
	// Normal
	copyModel.update_face_normals();
	copyModel.update_vertex_normals();
	Vector3VertexProperty new_normals = copyModel.vertex_property<Vector3d>(VNORMAL);
	foreach(Vertex v, copyModel.vertices()) sampleNormals.push_back( new_normals[v].cast<float>() );

	// Maybe add face centers + edge centers ?
	//foreach(Face f, copyModel.faces()){
	//	Vector3f sum(0); int c = 0;
	//	Surface_mesh::Vertex_around_face_circulator vit = copyModel.vertices(f),vend=vit;
	//	do{ sum += new_points[vit]; c++; } while(++vit != vend);
	//	samplePoints.push_back( sum / c );
	//}

	//foreach(Edge e, copyModel.edges()) 
	//	samplePoints.push_back( (new_points[copyModel.vertex(e,0)]+new_points[copyModel.vertex(e,1)]) / 2.0 );

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints, sampleNormals);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints, sampleNormals);
}

QVector<ParameterCoord> Synthesizer::genUniformTrisCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Sample mesh surface
	QVector<Vector3d> sample_normals;
	QVector<Vector3d> sample_points = SimilarSampler::All( model, uniformTriCount, sample_normals );
	
	// Double to float...
	std::vector<Vector3f> samplePointsF,sampleNormalsF;
	foreach(Vector3d p, sample_points) 
	{
		samplePointsF.push_back(p.cast<float>());
	}
	foreach(Vector3d n, sample_normals) 
	{
		sampleNormalsF.push_back(n.cast<float>());
	}

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePointsF, sampleNormalsF);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePointsF, sampleNormalsF);
}

// Generate rays depending on configuration of sampling types \s
QVector<ParameterCoord> Synthesizer::genSampleCoordsCurve( Structure::Curve * curve, int s )
{
	QVector<ParameterCoord> samples;

//	if (!curve->property.contains("original_sheet"))
	if (true)
	{
		if (s & Synthesizer::Features)	samples += genFeatureCoords(curve);
		if (s & Synthesizer::Edges)		samples += genEdgeCoords(curve);
		if (s & Synthesizer::Random)	samples += genRandomCoords(curve, randomCount);
		if (s & Synthesizer::Uniform)	samples += genUniformCoords(curve);
		if (s & Synthesizer::Remeshing)	samples += genRemeshCoords(curve);
		if (s & Synthesizer::TriUniform)samples += genUniformTrisCoords(curve);
	}
	else
	{
		// get samples using the original sheet
		Structure::Sheet* originalSheet  = curve->property["original_sheet"].value<Structure::Sheet*>();
		samples = genSampleCoordsSheet(originalSheet, s);

		// adjust the coordinates
		Array1D_Vector4d coords;
		RMF rmf = consistentFrame(curve,coords);
		QString curveDirection = originalSheet->property["curveDirection"].toString();
		for (int i = 0; i < (int)samples.size(); i++)
		{
			ParameterCoord &sample = samples[i];

			// adjust u, which is also t for curve
			//if (curveDirection == "positiveU");
			if (curveDirection == "negativeU")	sample.u = 1 - sample.u;
			if (curveDirection == "positiveV")	sample.u = sample.v;
			if (curveDirection == "positiveV")	sample.u = 1 - sample.v;

			// the corresponding frame on curve
			int idx = sample.u * (rmf.count() - 1);
			Vector3f X = rmf.U[idx].r.cast<float>();
			Vector3f Y = rmf.U[idx].s.cast<float>();
			Vector3f Z = rmf.U[idx].t.cast<float>();

			// encode the sample point in this frame
			float theta, psi;
			Vector3f delta = Vector3f(sample.origPoint - rmf.U[idx].center.cast<float>());
			Vector3f raydirection = delta.normalized();
			globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

			// update
			sample.theta = theta;
			sample.psi = psi;
			sample.origOffset = delta.norm();
			sample.origNode = curve;
		}
	}

	return samples;
}

QVector<ParameterCoord> Synthesizer::genSampleCoordsSheet( Structure::Sheet * sheet, int s )
{
	QVector<ParameterCoord> samples;

	if (s & Synthesizer::Features)	samples += genFeatureCoords(sheet);
	if (s & Synthesizer::Edges)		samples += genEdgeCoords(sheet);
	if (s & Synthesizer::Random)	samples += genRandomCoords(sheet, randomCount);
	if (s & Synthesizer::Uniform)	samples += genUniformCoords(sheet);
	if (s & Synthesizer::Remeshing)	samples += genRemeshCoords(sheet);
	if (s & Synthesizer::TriUniform)samples += genUniformTrisCoords(sheet);
	
	return samples;
}

// Compute offset and normal for each ray
void Synthesizer::sampleGeometryCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QVector<float> &offsets, QVector<Vec2f> &normals )
{
	SurfaceMesh::Model * model = curve->property["mesh"].value<SurfaceMesh::Model*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vector3d>("f:normal");
    Octree octree(model, OCTREE_NODE_SIZE);

	offsets.clear();
	offsets.resize(samples.size());

	normals.clear();
	normals.resize(samples.size());

	const ParameterCoord * samplesArray = samples.data();

	// Generate consistent frames along curve
	Array1D_Vector4d coords;
	RMF rmf = consistentFrame(curve,coords);
	qDebug() << "Curve RMF count = " << rmf.U.size() << ", Samples = " << samples.size();

	const std::vector<Vector3d> curvePnts = curve->curve.mCtrlPoint;
	int N = samples.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSCurved mycurve = NURBS::NURBSCurved::createCurveFromPoints(curvePnts);

		ParameterCoord sample = samplesArray[i];
		
		int idx = sample.u * (rmf.count() - 1);
		Vector3f X = rmf.U[idx].r.normalized().cast<float>();
		Vector3f Y = rmf.U[idx].s.normalized().cast<float>();
		Vector3f Z = rmf.U[idx].t.normalized().cast<float>();

		Vector4d coordinate(sample.u,0,0,0);
		Vector3f rayPos = mycurve.GetPosition( coordinate[0] ).cast<float>();
		Vector3f rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Vector3d rayPosition(rayPos.x(), rayPos.y(), rayPos.z());
		Vector3d rayDirection(rayDir.x(), rayDir.y(), rayDir.z());

		Ray ray( rayPosition, rayDirection );
		int faceIndex = 0;

		Vector3f vn(1,1,1);

		if(curve == sample.origNode)
		{
			offsets[ i ] = sample.origOffset;
			vn = sample.origNormal;
		}
		else
		{
			Vector3 isect = octree.closestIntersectionPoint(ray, &faceIndex);

			// Store the offset
			offsets[ i ] = (Vector3(isect - rayPos.cast<double>())).norm();

			vn = fnormals[ SurfaceMesh::Model::Face(faceIndex) ].cast<float>();
		}

		// Code the normal relative to local frame
		Vec2f normalCoord(0,0);
		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], vn);

		normals[ i ] = normalCoord;
	}
}

void Synthesizer::sampleGeometrySheet( QVector<ParameterCoord> samples, Structure::Sheet * sheet, QVector<float> &offsets, QVector<Vec2f> &normals )
{
	SurfaceMesh::Model * model = sheet->property["mesh"].value<SurfaceMesh::Model*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vector3d>("f:normal");
    Octree octree(model, OCTREE_NODE_SIZE);

	offsets.clear();
	offsets.resize( samples.size() );

	normals.clear();
	normals.resize( samples.size() );

	qDebug() << "Sheet samples = " << samples.size();

	const ParameterCoord * samplesArray = samples.data();

	const Array2D_Vector3 sheetPnts = sheet->surface.mCtrlPoint;
	int N = samples.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSRectangled r = NURBS::NURBSRectangled::createSheetFromPoints(sheetPnts);

		ParameterCoord sample = samplesArray[i];

		Vector3d X(0,0,0), Y(0,0,0), Z(0,0,0);
		Vector3d vDirection(0,0,0), rayPos(0,0,0);

		r.GetFrame( sample.u, sample.v, rayPos, X, vDirection, Z );
		Y = cross(Z, X);

		Vector3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		Vector3d vn(1,1,1);

		if(sheet == sample.origNode)
		{
			offsets[i] = sample.origOffset;
			vn = sample.origNormal.cast<double>();
		}
		else
		{
			// Store the offset
			Vector3 isect = octree.closestIntersectionPoint(ray, &faceIndex);
			offsets[i] = (isect - rayPos).norm();

			// Code the normal relative to local frame
			vn = fnormals[SurfaceMesh::Model::Face(faceIndex)];
		}

		Vec2f normalCoord;
		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], vn);
		normals[i] = normalCoord;
	}

	qDebug() << QString("Sheet [%1] Done.").arg(sheet->id);
}

// Co-sampling
void Synthesizer::prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2, int s )
{
	if(!curve1 || !curve2 || !curve1->property.contains("mesh") || !curve2->property.contains("mesh")) return;

	QElapsedTimer timer; timer.start();
	qDebug() << "Generating samples..";

	QVector<ParameterCoord> samples;
	QVector<float> offsets1, offsets2;
	QVector<Vec2f> normals1, normals2;

	// Sample two curves to get rays
	{
		if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
		if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

		int old_uniformTriCount = uniformTriCount;
		if (curve1->property.contains("original_sheet") || curve2->property.contains("original_sheet"))
		{
			// Super sample converted parts
			uniformTriCount *= 10;
		}

		samples  = genSampleCoordsCurve(curve1, s);
		samples += genSampleCoordsCurve(curve2, s);

		if (curve1->property.contains("original_sheet") || curve2->property.contains("original_sheet"))
		{
			// Reset the sampling rate
			uniformTriCount = old_uniformTriCount;
		}

		// Why need sorting? -HH
		// Sort samples by 'u'
		sort(samples.begin(), samples.end());
	}

	qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());timer.restart();

	qDebug() << "Re-sampling mesh..";

	// Compute offset and normal for each ray
	{
		sampleGeometryCurve(samples, curve1, offsets1, normals1);
		curve1->property["samples"].setValue(samples);
		curve1->property["offsets"].setValue(offsets1);
		curve1->property["normals"].setValue(normals1);

		sampleGeometryCurve(samples, curve2, offsets2, normals2);
		curve2->property["samples"].setValue(samples);
		curve2->property["offsets"].setValue(offsets2);
		curve2->property["normals"].setValue(normals2);
	}

	qDebug() << QString("Resampling Time [ %1 ms ]\n==\n").arg(timer.elapsed());
}

void Synthesizer::prepareSynthesizeSheet( Structure::Sheet * sheet1, Structure::Sheet * sheet2, int s )
{
	if(!sheet1 || !sheet2 || !sheet1->property.contains("mesh") || !sheet2->property.contains("mesh")) return;

	QElapsedTimer timer; timer.start();
	qDebug() << "Generating samples..";

	QVector<ParameterCoord> samples;
	QVector<float> offsets1, offsets2;
	QVector<Vec2f> normals1, normals2;
	//bool c1_isSampled = false, c2_isSampled = false;

	// Sample two sheets
	{
		if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
		if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

		samples  = genSampleCoordsSheet(sheet1, s);
		if(sheet1 != sheet2) samples += genSampleCoordsSheet(sheet2, s);
	}

	qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());timer.restart();

	qDebug() << "Re-sampling mesh..";

	// Re-sample the meshes
	{	
		sampleGeometrySheet(samples, sheet1, offsets1, normals1);
		sheet1->property["samples"].setValue(samples);
		sheet1->property["offsets"].setValue(offsets1);
		sheet1->property["normals"].setValue(normals1);

		if(sheet1 != sheet2) 
		{
			sampleGeometrySheet(samples, sheet2, offsets2, normals2);
			sheet2->property["samples"].setValue(samples);
			sheet2->property["offsets"].setValue(offsets2);
			sheet2->property["normals"].setValue(normals2);
		}
	}

	qDebug() << QString("Resampling Time [ %1 ms ]\n==\n").arg(timer.elapsed());
}



/// BLENDING
void Synthesizer::blendCurveBases( Structure::Curve * curve1, Structure::Curve * curve2, float alpha )
{
	std::vector<Vector3d> &ctrlPnts1 = curve1->curve.mCtrlPoint;

	for (int i = 0; i < (int) ctrlPnts1.size(); i++)
	{
		Vector3d cp1 = ctrlPnts1[i];
		float time = float(i) / (ctrlPnts1.size() - 1);
		Vector3d cp2 = curve2->curve.GetPosition(time);

		ctrlPnts1[i] = (1 - alpha) * cp1 + alpha * cp2;
	}
}

void Synthesizer::blendSheetBases( Structure::Sheet * sheet1, Structure::Sheet * sheet2, float alpha )
{
    sheet1 = sheet1;
    sheet2 = sheet2;
    alpha = alpha;
}

void Synthesizer::blendGeometryCurves( Structure::Curve * curve1, Structure::Curve * curve2, float alpha, QVector<Vector3f> &points, QVector<Vector3f> &normals )
{
	if(!curve1->property.contains("samples") || !curve2->property.contains("samples")) return;
	alpha = qMax(0.0f, qMin(alpha, 1.0f));

	QVector<ParameterCoord> samples = curve1->property["samples"].value< QVector<ParameterCoord> >();

	QVector<float> offsets1 = curve1->property["offsets"].value< QVector<float> >();
	QVector<float> offsets2 = curve2->property["offsets"].value< QVector<float> >();

	QVector<Vec2f> normals1 = curve1->property["normals"].value< QVector<Vec2f> >();
	QVector<Vec2f> normals2 = curve2->property["normals"].value< QVector<Vec2f> >();

	// Blend Geometries 
	QVector<float> blended_offsets;
	QVector<Vec2f> blended_normals;
	for( int i = 0; i < offsets1.size(); i++)
	{
		float off = ((1 - alpha) * offsets1[i]) + (alpha * offsets2[i]);
		blended_offsets.push_back( off );

		Vec2f n = ((1 - alpha) * normals1[i]) + (alpha * normals2[i]);
		blended_normals.push_back( n );
	}

	// Reconstruct geometry on the new base
	reconstructGeometryCurve(curve1, samples, blended_offsets, blended_normals, points, normals);
}

void Synthesizer::blendGeometrySheets( Structure::Sheet * sheet1, Structure::Sheet * sheet2, float alpha, QVector<Vector3f> &points, QVector<Vector3f> &normals )
{
	if(!sheet1->property.contains("samples") || !sheet2->property.contains("samples")) return;

	QVector<ParameterCoord> samples = sheet1->property["samples"].value< QVector<ParameterCoord> >();

	QVector<float> offsets1 = sheet1->property["offsets"].value< QVector<float> >();
	QVector<float> offsets2 = sheet2->property["offsets"].value< QVector<float> >();

	QVector<Vec2f> normals1 = sheet1->property["normals"].value< QVector<Vec2f> >();
	QVector<Vec2f> normals2 = sheet2->property["normals"].value< QVector<Vec2f> >();

	// Blend Geometries 
	QVector<float> blended_offsets;
	QVector<Vec2f> blended_normals;
	for( int i = 0; i < offsets1.size(); i++)
	{
		float off = ((1 - alpha) * offsets1[i]) + (alpha * offsets2[i]);
		blended_offsets.push_back( off );

		Vec2f n = ((1 - alpha) * normals1[i]) + (alpha * normals2[i]);
		blended_normals.push_back( n );
	}
	// Reconstruct geometry on the new base
	reconstructGeometrySheet(sheet1, samples, blended_offsets, blended_normals, points, normals);
}

/// RECONSTRUCTION
void Synthesizer::reconstructGeometryCurve( Structure::Curve * base_curve, QVector<ParameterCoord> in_samples, QVector<float> &in_offsets,
	QVector<Vec2f> &in_normals, QVector<Vector3f> &out_points, QVector<Vector3f> &out_normals )
{
	// Clear
	out_points.clear();
	out_points.resize(in_samples.size());
	out_normals.clear();
	out_normals.resize(in_samples.size());

	// Generate consistent frames along curve
	Array1D_Vector4d coords;
	RMF rmf = consistentFrame(base_curve,coords);

	for(int i = 0; i < (int) in_samples.size(); i++)
	{
		ParameterCoord sample = in_samples[i];

		Vector3f rayPos, rayDir;

		int idx = sample.u * (rmf.count() - 1);

		Vector3f X = rmf.U[idx].r.normalized().cast<float>();
		Vector3f Y = rmf.U[idx].s.normalized().cast<float>();
		Vector3f Z = rmf.U[idx].t.normalized().cast<float>();

		rayPos = base_curve->position(Vector4d(sample.u,0,0,0)).cast<float>();

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);

		// Reconstructed point
		Vector3f isect = rayPos + (rayDir.normalized() * in_offsets[ i ]);
		out_points[ i ] = isect;

		// Reconstructed normal
		Vec2f normalCoord = in_normals[ i ];
		Vector3f normal(0,0,0);
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);

		// Normal correction
		{
			float theta = acos(qRanged(-1.0f,(float)dot(normal.normalized(),rayDir.normalized()),1.0f));
			if(theta > M_PI / 2.0) normal = rayDir;
		}

		out_normals[ i ] = normal.normalized();
	}
}

void Synthesizer::reconstructGeometrySheet( Structure::Sheet * base_sheet,  QVector<ParameterCoord> in_samples, QVector<float> &in_offsets,
	QVector<Vec2f> &in_normals, QVector<Vector3f> &out_points, QVector<Vector3f> &out_normals )
{
	out_points.clear();
	out_points.resize(in_samples.size());

	out_normals.clear();
	out_normals.resize(in_samples.size());

	int N = in_samples.size();

	for(int i = 0; i < N; i++)
	{
		Vector3d _X, _Y, _Z;
		Vector3d vDirection, sheetPoint;

		Vector3f rayPos, rayDir;

		ParameterCoord sample = in_samples[i];

		base_sheet->surface.GetFrame( sample.u, sample.v, sheetPoint, _X, vDirection, _Z );
		_Y = cross(_Z, _X);

		rayPos = base_sheet->position(Vector4d(sample.u, sample.v, 0, 0)).cast<float>();

		// double float
		Vector3f X = _X.cast<float>(), Y = _Y.cast<float>(), Z = _Z.cast<float>();

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);
		rayDir.normalize();

		// Reconstructed point
		Vector3f isect = rayPos + (rayDir * in_offsets[i]);
		out_points[i] = isect;

		// Reconstructed normal
		Vec2f normalCoord = in_normals[i];
		Vector3f normal;
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);

		// Normal correction
		{
			float theta = acos(qRanged(-1.0f,(float)dot(normal.normalized(),rayDir),1.0f));
			if(theta > M_PI / 2.0) normal = rayDir;
		}

		out_normals[i] = normal.normalized();
	}
}



/// I/O and copy
void Synthesizer::copySynthData( Structure::Node * fromNode, Structure::Node * toNode )
{
	toNode->property["samples"].setValue( fromNode->property["samples"].value< QVector<ParameterCoord> >() );
	toNode->property["offsets"].setValue( fromNode->property["offsets"].value< QVector<float> >() );
	toNode->property["normals"].setValue( fromNode->property["normals"].value< QVector<Vec2f> >() );
}

void Synthesizer::clearSynthData( Structure::Node * fromNode )
{
	fromNode->property.remove("samples");
	fromNode->property.remove("offsets");
	fromNode->property.remove("normals");
	
	// Visualization
	fromNode->property.remove("cached_points");
	fromNode->property.remove("cached_normals");
}

void Synthesizer::writeXYZ( QString filename, std::vector<Vector3f> points, std::vector<Vector3f> normals )
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	for(int i = 0; i < (int) points.size(); i++)
	{
		Vector3f p = points[i];
		Vector3f n = normals[i];
		out << QString("%1 %2 %3 %4 %5 %6\n").arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]);
	}

	file.close();
}

void Synthesizer::saveSynthesisData( Structure::Node *node, QString prefix )
{
	if(!node || !node->property.contains("samples")) return;

	QVector<ParameterCoord> samples = node->property["samples"].value< QVector<ParameterCoord> >();
	QVector<float> offsets = node->property["offsets"].value< QVector<float> >();
	QVector<Vec2f> normals = node->property["normals"].value< QVector<Vec2f> >();

	if(samples.empty())
	{
		qDebug() << QString("WARNING: Node [%1]: No data").arg(node->id);
		return;
	}

	QFile file(prefix + node->id + ".txt");
	if (!file.open(QIODevice::WriteOnly)) return;
	QDataStream out(&file);

	out << samples.size();

	for(int i = 0; i < (int) samples.size(); i++)
	{
		out << samples[i].u << samples[i].v << samples[i].theta << samples[i].psi << 
			offsets[i] << normals[i][0] << normals[i][1];
	}

	file.close();
}

void Synthesizer::loadSynthesisData( Structure::Node *node, QString prefix )
{
	if(!node) return;

	QFile file(prefix + node->id + ".txt");
	if (!file.open(QIODevice::ReadOnly)) return;
	QDataStream inF(&file);

	int num;
	inF >> num;

	QVector<ParameterCoord> samples(num);
	QVector<float> offsets(num);
	QVector<Vec2f> normals(num);

	for(int i = 0; i < num; i++)
	{
		inF >> samples[i].u >> samples[i].v >> samples[i].theta >> samples[i].psi >> 
			offsets[i] >> normals[i][0] >> normals[i][1];
	}

	node->property["samples"].setValue(samples);
	node->property["offsets"].setValue(offsets);
	node->property["normals"].setValue(normals);
}
