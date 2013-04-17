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

typedef std::pair<ParameterCoord,int> ParameterCoordInt;
bool comparatorParameterCoordInt ( const ParameterCoordInt& l, const ParameterCoordInt& r)
{ return l.first.u < r.first.u; }

// Parameters
#define CURVE_FRAME_RESOLUTION 0.01
#define SHEET_FRAME_RESOLUTION 0.01
#define CURVE_FRAME_COUNT 100

#define OCTREE_NODE_SIZE 40

// Sampling
#define RANDOM_COUNT 1e3
#define UNIFORM_RESOLUTION 0.01
#define EDGE_RESOLUTION 0.05
#define UNIFORM_TRI_SAMPLES 1e4

// Remeshing
#define REMESH_EDGE_LENGTH 0.005

int global_countt = 0;

int randomCount = RANDOM_COUNT;
int uniformTriCount = UNIFORM_TRI_SAMPLES;

// Helper functions
void Synthesizer::sortSamplesCurve( QVector<ParameterCoord> & samples, QVector<int> & oldIndices )
{
	std::vector< ParameterCoordInt > paramIndex;

	foreach( ParameterCoord pc, samples )
		paramIndex.push_back( std::make_pair( pc, (int)paramIndex.size() ) );

	std::sort( paramIndex.begin(), paramIndex.end(), comparatorParameterCoordInt );

	QVector<ParameterCoord> sorted_samples;
	foreach(ParameterCoordInt pci, paramIndex)
	{
		oldIndices.push_back(pci.second);
		sorted_samples.push_back(pci.first);
	}

	samples = sorted_samples;
}

RMF Synthesizer::consistentFrame( Structure::Curve * curve, Array1D_Vec4d & coords )
{
	// Generate consistent frames along curve
	std::vector<Scalar> times;
	curve->curve.SubdivideByLengthTime(CURVE_FRAME_COUNT, times);
	foreach(Scalar t, times) coords.push_back(Vec4d(t,0,0,0));
	std::vector<Vec3d> samplePoints;
	foreach(Vec4d c, coords) samplePoints.push_back( curve->position(c) );

    RMF rmf = RMF( samplePoints, false );
	rmf.generate();

    curve->property["consistentFrame"].setValue( rmf );
	return rmf;
}

QVector<ParameterCoord> Synthesizer::genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points )
{
	QVector<ParameterCoord> samples(points.size());

	// Generate consistent frames along curve
	Array1D_Vec4d coords;
	RMF rmf = consistentFrame(curve,coords);

	// Add all curve points to kd-tree
	NanoKdTree kdtree;
	foreach(Vec3d p, rmf.point) kdtree.addPoint( p );
	kdtree.build();

	// Project
	const std::vector<Vector3> curvePnts = curve->curve.mCtrlPoint;
	int N = points.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSCurved mycurve = NURBS::NURBSCurved::createCurveFromPoints( curvePnts );

		double theta, psi;

		Vec3d point = points[i];

		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = coords[closest_idx];

		Vec3d X = rmf.U[closest_idx].r.normalized();
		Vec3d Y = rmf.U[closest_idx].s.normalized();
		Vec3d Z = rmf.U[closest_idx].t.normalized();

		Vec3d curvePoint = mycurve.GetPosition(c[0]);
		Vec3d raydirection = (point - curvePoint).normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

		samples[i] = ParameterCoord(theta, psi, c[0]);
	}

	return samples;
}

QVector<ParameterCoord> Synthesizer::genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vec3d> & points )
{
	QVector<ParameterCoord> samples(points.size());

	// Auto resolution
	double resolution = sheet->bbox().size().length() * SHEET_FRAME_RESOLUTION;

	Array2D_Vec4d sheetCoords = sheet->discretizedPoints(resolution);
	Array1D_Vec4d allCoords;

	qDebug() << "Sheet resolution count = " << sheetCoords.size();

	// Add all curve points to kd-tree
	NanoKdTree kdtree;
	foreach(Array1D_Vec4d row, sheetCoords)
	{
		foreach(Vec4d c, row) 
		{
			kdtree.addPoint( sheet->position(c) );
			allCoords.push_back(c);
		}
	}
	kdtree.build();

	const Array2D_Vector3 sheetPnts = sheet->surface.mCtrlPoint;
	int N = points.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSRectangled r = NURBS::NURBSRectangled::createSheetFromPoints(sheetPnts);

		Vec3d point = points[i];

		// Project
		Vector3 X, Y, Z;
		Vector3 vDirection, sheetPoint;

		double theta, psi;

		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = allCoords[closest_idx];

		r.GetFrame( c[0], c[1], sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);
		
		Vec3d raydirection = (point - sheetPoint).normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

		samples[i] = ParameterCoord(theta, psi, c[0], c[1]);
	}

    return samples;
}

QVector<ParameterCoord> Synthesizer::genFeatureCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
	std::vector<Vec3d> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v]);

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, meshPoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, meshPoints);
}

QVector<ParameterCoord> Synthesizer::genEdgeCoords( Structure::Node * node, double sampling_resolution /*= -1 */ )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");

	// Auto resolution
	if(sampling_resolution < 0) sampling_resolution = node->bbox().size().length() * EDGE_RESOLUTION;

	// Sample mesh surface
	std::vector<Vec3d> samplePoints;
	typedef QPair<Vec3d, Vec3d> QPairVec3d;
	QVector< QPairVec3d > meshEdges;

	// Collect mesh edges
	foreach(Edge e, model->edges())
		meshEdges.push_back( qMakePair(points[model->vertex(e, 0)],points[model->vertex(e, 1)]) );

	// Sample edges up to sampling resolution
	foreach(QPairVec3d edge, meshEdges){
		for(double t = 0; t <= 1; t += sampling_resolution)
		{
			samplePoints.push_back( ((1 - t) * edge.first) + (t * edge.second) );
		}
	}

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genRandomCoords(Node *node, int samples_count)
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Sample mesh surface
	std::vector<Vec3d> samplePoints;
	samplePoints = SpherePackSampling::getRandomSamples(model, samples_count);

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genUniformCoords(Node *node, double sampling_resolution)
{
    SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

    // Auto resolution
    if(sampling_resolution < 0) sampling_resolution = UNIFORM_RESOLUTION;

    // Sample mesh surface
    std::vector<Vec3d> samplePoints, gp;
    samplePoints = SpherePackSampling::sample(model, 1e4, sampling_resolution, gp, 1);

    if(node->type() == Structure::CURVE)
        return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
    else
        return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genRemeshCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();
	SurfaceMesh::Model copyModel;

	std::vector<Vec3d> samplePoints;

	Vector3VertexProperty points = model->vertex_property<Vector3>(VPOINT);

	foreach(Vertex v, model->vertices()) copyModel.add_vertex( points[v] );
	foreach(Face f, model->faces()){
		std::vector<Vertex> verts;
		Surface_mesh::Vertex_around_face_circulator vit = model->vertices(f),vend=vit;
		do{ verts.push_back(vit); } while(++vit != vend);
		copyModel.add_face(verts);
	}

	IsotropicRemesher remesher(&copyModel);
	remesher.doRemesh( REMESH_EDGE_LENGTH );
	
	Vector3VertexProperty new_points = copyModel.vertex_property<Vector3>(VPOINT);
	foreach(Vertex v, copyModel.vertices()) samplePoints.push_back( new_points[v] );

	// Maybe add face centers + edge centers ?
	//foreach(Face f, copyModel.faces()){
	//	Vec3d sum(0); int c = 0;
	//	Surface_mesh::Vertex_around_face_circulator vit = copyModel.vertices(f),vend=vit;
	//	do{ sum += new_points[vit]; c++; } while(++vit != vend);
	//	samplePoints.push_back( sum / c );
	//}

	//foreach(Edge e, copyModel.edges()) 
	//	samplePoints.push_back( (new_points[copyModel.vertex(e,0)]+new_points[copyModel.vertex(e,1)]) / 2.0 );

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genUniformTrisCoords( Structure::Node * node )
{
	SurfaceMesh::Model * model = node->property["mesh"].value<SurfaceMesh::Model*>();

	// Sample mesh surface
	QVector<Vec3d> sample_points = SimilarSampler::All( model, uniformTriCount );
	std::vector<Vec3d> samplePoints = sample_points.toStdVector();

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

void Synthesizer::sampleGeometryCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QVector<double> &offsets, QVector<Vec2d> &normals )
{
	SurfaceMesh::Model * model = curve->property["mesh"].value<SurfaceMesh::Model*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
    Octree octree(model, OCTREE_NODE_SIZE);

	offsets.clear();
	offsets.resize(samples.size());

	normals.clear();
	normals.resize(samples.size());

	const ParameterCoord * samplesArray = samples.data();

	// Generate consistent frames along curve
	Array1D_Vec4d coords;
	RMF rmf = consistentFrame(curve,coords);
	qDebug() << "Curve RMF count = " << rmf.U.size() << ", Samples = " << samples.size();

	const std::vector<Vector3> curvePnts = curve->curve.mCtrlPoint;
	int N = samples.size();

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		NURBS::NURBSCurved mycurve = NURBS::NURBSCurved::createCurveFromPoints(curvePnts);

		ParameterCoord sample = samplesArray[i];
		
		int idx = sample.u * (rmf.count() - 1);
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		Vec4d coordinate(sample.u);
		Vec3d rayPos = mycurve.GetPosition( coordinate[0] );
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

        Vec3d isect = octree.closestIntersectionPoint(ray, &faceIndex);

		// Store the offset
		offsets[ i ] = (isect - rayPos).norm();

		// Code the normal relative to local frame
		Vec2d normalCoord(0);
		Vec3d v = fnormals[ SurfaceMesh::Model::Face(faceIndex) ];

		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], v);

		normals[ i ] = normalCoord;
	}
}

void Synthesizer::sampleGeometrySheet( QVector<ParameterCoord> samples, Structure::Sheet * sheet, QVector<double> &offsets, QVector<Vec2d> &normals )
{
	SurfaceMesh::Model * model = sheet->property["mesh"].value<SurfaceMesh::Model*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
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

		Vector3 X(0), Y(0), Z(0);
		Vector3 vDirection(0), rayPos(0);

		r.GetFrame( sample.u, sample.v, rayPos, X, vDirection, Z );
		Y = cross(Z, X);

		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		// Store the offset
        Vec3d isect = octree.closestIntersectionPoint(ray, &faceIndex);
		offsets[i] = (isect - rayPos).norm();

		// Code the normal relative to local frame
		Vec2d normalCoord;
		Vec3d v = fnormals[SurfaceMesh::Model::Face(faceIndex)];

		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], v);
		normals[i] = normalCoord;
	}

	qDebug() << QString("Sheet [%1] Done.").arg(sheet->id);
}


void Synthesizer::blendCurveBases( Structure::Curve * curve1, Structure::Curve * curve2, double alpha )
{
	std::vector<Vector3> &ctrlPnts1 = curve1->curve.mCtrlPoint;

	for (int i = 0; i < (int) ctrlPnts1.size(); i++)
	{
		Vector3 cp1 = ctrlPnts1[i];
		double time = double(i) / (ctrlPnts1.size() - 1);
		Vector3 cp2 = curve2->curve.GetPosition(time);

		ctrlPnts1[i] = (1 - alpha) * cp1 + alpha * cp2;
	}
}

void Synthesizer::blendSheetBases( Structure::Sheet * sheet1, Structure::Sheet * sheet2, double alpha )
{
    sheet1 = sheet1;
    sheet2 = sheet2;
    alpha = alpha;
}

void Synthesizer::reconstructGeometryCurve( Structure::Curve * base_curve, QVector<ParameterCoord> in_samples, QVector<double> &in_offsets,
	QVector<Vec2d> &in_normals, QVector<Vector3> &out_points, QVector<Vector3> &out_normals )
{
	// Clear
	out_points.clear();
	out_points.resize(in_samples.size());
	out_normals.clear();
	out_normals.resize(in_samples.size());

	// Generate consistent frames along curve
	Array1D_Vec4d coords;
	RMF rmf = consistentFrame(base_curve,coords);

	for(int i = 0; i < (int) in_samples.size(); i++)
	{
		ParameterCoord sample = in_samples[i];

		Vec3d rayPos, rayDir;

		int idx = sample.u * (rmf.count() - 1);

		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		rayPos = base_curve->position(Vec4d(sample.u));

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);

		// Reconstructed point
		Vec3d isect = rayPos + (rayDir.normalized() * in_offsets[ i ]);
		out_points[ i ] = isect;

		// Reconstructed normal
		Vec2d normalCoord = in_normals[ i ];
		Vec3d normal(0);
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);

		// Normal correction
		{
			double theta = acos(qRanged(-1.0,dot(normal.normalized(),rayDir.normalized()),1.0));
			if(theta > M_PI / 2.0) normal = rayDir;
		}

		out_normals[ i ] = normal.normalized();
	}

	// Save RMF frames
	base_curve->property["rmf_frames"].setValue(rmf.U);
}

void Synthesizer::reconstructGeometrySheet( Structure::Sheet * base_sheet,  QVector<ParameterCoord> in_samples, QVector<double> &in_offsets,
	QVector<Vec2d> &in_normals, QVector<Vector3> &out_points, QVector<Vector3> &out_normals )
{
	out_points.clear();
	out_points.resize(in_samples.size());

	out_normals.clear();
	out_normals.resize(in_samples.size());

	int N = in_samples.size();

	for(int i = 0; i < N; i++)
	{
		Vector3 X, Y, Z;
		Vector3 vDirection, sheetPoint;

		Vec3d rayPos, rayDir;

		ParameterCoord sample = in_samples[i];

		base_sheet->surface.GetFrame( sample.u, sample.v, sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);

		rayPos = base_sheet->position(Vec4d(sample.u, sample.v, 0, 0));

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);
		rayDir.normalize();

		// Reconstructed point
		Vec3d isect = rayPos + (rayDir * in_offsets[i]);
		out_points[i] = isect;

		// Reconstructed normal
		Vec2d normalCoord = in_normals[i];
		Vec3d normal;
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);

		// Normal correction
		{
			double theta = acos(qRanged(-1.0,dot(normal.normalized(),rayDir),1.0));
			if(theta > M_PI / 2.0) normal = rayDir;
		}

		out_normals[i] = normal.normalized();
	}
}

void Synthesizer::prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2, int s )
{
	if(!curve1 || !curve2 || !curve1->property.contains("mesh") || !curve2->property.contains("mesh")) return;

	QElapsedTimer timer; timer.start();
	qDebug() << "Generating samples..";
	
	QVector<ParameterCoord> samples;
	QVector<double> offsets1, offsets2;
	QVector<Vec2d> normals1, normals2;
	bool c1_isSampled = false, c2_isSampled = false;

	// Skip sampled by using own samples and copying them to sampled
	if(curve1->property.contains("samples")){
		samples = curve1->property["samples"].value< QVector<ParameterCoord> >();
		c1_isSampled = true;
	}
	if(curve2->property.contains("samples")){
		samples = curve2->property["samples"].value< QVector<ParameterCoord> >();
		c2_isSampled = true;
	}

	// Sample two curves
	if( !samples.size() )
	{
		if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
		if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

		if (s & Synthesizer::Features)	samples += genFeatureCoords(curve1) + genFeatureCoords(curve2);
		if (s & Synthesizer::Edges)		samples += genEdgeCoords(curve1) + genEdgeCoords(curve2);
		if (s & Synthesizer::Random)	samples += genRandomCoords(curve1, randomCount) + genRandomCoords(curve2, randomCount);
		if (s & Synthesizer::Uniform)	samples += genUniformCoords(curve1) + genUniformCoords(curve2);
		if (s & Synthesizer::Remeshing)	samples += genRemeshCoords(curve1) + genRemeshCoords(curve2);
		if (s & Synthesizer::TriUniform)samples += genUniformTrisCoords(curve1) + genUniformTrisCoords(curve2);

		// Sort samples by 'u'
		sort(samples.begin(), samples.end());
	}

	qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());timer.restart();

	qDebug() << "Re-sampling mesh..";

	// Re-sample the meshes
	if( !c1_isSampled ) 
	{	
		sampleGeometryCurve(samples, curve1, offsets1, normals1);
		curve1->property["samples"].setValue(samples);
		curve1->property["offsets"].setValue(offsets1);
		curve1->property["normals"].setValue(normals1);
	}

	if( !c2_isSampled ) 
	{
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
	QVector<double> offsets1, offsets2;
	QVector<Vec2d> normals1, normals2;
	bool c1_isSampled = false, c2_isSampled = false;

	// Skip sampled by using own samples and copying them to sampled
	if(sheet1->property.contains("samples")){
		samples = sheet1->property["samples"].value< QVector<ParameterCoord> >();
		c1_isSampled = true;
	}
	if(sheet2->property.contains("samples")){
		samples = sheet2->property["samples"].value< QVector<ParameterCoord> >();
		c2_isSampled = true;
	}

	// Sample two sheets
	if( !samples.size() )
	{
		if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
		if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

		if (s & Synthesizer::Features)	samples += genFeatureCoords(sheet1) + genFeatureCoords(sheet2);
		if (s & Synthesizer::Edges)		samples += genEdgeCoords(sheet1) + genEdgeCoords(sheet2);
		if (s & Synthesizer::Random)	samples += genRandomCoords(sheet1, randomCount) + genRandomCoords(sheet2, randomCount);
		if (s & Synthesizer::Uniform)	samples += genUniformCoords(sheet1) + genUniformCoords(sheet2);
		if (s & Synthesizer::Remeshing)	samples += genRemeshCoords(sheet1) + genRemeshCoords(sheet2);
		if (s & Synthesizer::TriUniform)samples += genUniformTrisCoords(sheet1) + genUniformTrisCoords(sheet2);
	}

	qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());timer.restart();

	qDebug() << "Re-sampling mesh..";

	// Re-sample the meshes
	if( !c1_isSampled ) 
	{	
		sampleGeometrySheet(samples, sheet1, offsets1, normals1);
		sheet1->property["samples"].setValue(samples);
		sheet1->property["offsets"].setValue(offsets1);
		sheet1->property["normals"].setValue(normals1);
	}

	if( !c2_isSampled ) 
	{
		sampleGeometrySheet(samples, sheet2, offsets2, normals2);
		sheet2->property["samples"].setValue(samples);
		sheet2->property["offsets"].setValue(offsets2);
		sheet2->property["normals"].setValue(normals2);
	}

	qDebug() << QString("Resampling Time [ %1 ms ]\n==\n").arg(timer.elapsed());
}

void Synthesizer::copySynthData( Structure::Node * fromNode, Structure::Node * toNode )
{
	toNode->property["samples"].setValue( fromNode->property["samples"].value< QVector<ParameterCoord> >() );
	toNode->property["offsets"].setValue( fromNode->property["offsets"].value< QVector<double> >() );
	toNode->property["normals"].setValue( fromNode->property["normals"].value< QVector<Vec2d> >() );
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

void Synthesizer::blendGeometryCurves( Structure::Curve * curve1, Structure::Curve * curve2, double alpha, QVector<Vector3> &points, QVector<Vector3> &normals )
{
	if(!curve1->property.contains("samples") || !curve2->property.contains("samples")) return;
	alpha = qMax(0.0, qMin(alpha, 1.0));

	QVector<ParameterCoord> samples = curve1->property["samples"].value< QVector<ParameterCoord> >();

	QVector<double> offsets1 = curve1->property["offsets"].value< QVector<double> >();
	QVector<double> offsets2 = curve2->property["offsets"].value< QVector<double> >();

	QVector<Vec2d> normals1 = curve1->property["normals"].value< QVector<Vec2d> >();
	QVector<Vec2d> normals2 = curve2->property["normals"].value< QVector<Vec2d> >();

	// Blend Geometries 
	QVector<double> blended_offsets;
	QVector<Vec2d> blended_normals;
	for( int i = 0; i < offsets1.size(); i++)
	{
		double off = ((1 - alpha) * offsets1[i]) + (alpha * offsets2[i]);
		blended_offsets.push_back( off );

		Vec2d n = ((1 - alpha) * normals1[i]) + (alpha * normals2[i]);
		blended_normals.push_back( n );
	}

	// Reconstruct geometry on the new base
	reconstructGeometryCurve(curve1, samples, blended_offsets, blended_normals, points, normals);
}

void Synthesizer::blendGeometrySheets( Structure::Sheet * sheet1, Structure::Sheet * sheet2, double alpha, QVector<Vector3> &points, QVector<Vector3> &normals )
{
	if(!sheet1->property.contains("samples") || !sheet2->property.contains("samples")) return;

	QVector<ParameterCoord> samples = sheet1->property["samples"].value< QVector<ParameterCoord> >();

	QVector<double> offsets1 = sheet1->property["offsets"].value< QVector<double> >();
	QVector<double> offsets2 = sheet2->property["offsets"].value< QVector<double> >();

	QVector<Vec2d> normals1 = sheet1->property["normals"].value< QVector<Vec2d> >();
	QVector<Vec2d> normals2 = sheet2->property["normals"].value< QVector<Vec2d> >();

	// Blend Geometries 
	QVector<double> blended_offsets;
	QVector<Vec2d> blended_normals;
	for( int i = 0; i < offsets1.size(); i++)
	{
		double off = ((1 - alpha) * offsets1[i]) + (alpha * offsets2[i]);
		blended_offsets.push_back( off );

		Vec2d n = ((1 - alpha) * normals1[i]) + (alpha * normals2[i]);
		blended_normals.push_back( n );
	}
	// Reconstruct geometry on the new base
	reconstructGeometrySheet(sheet1, samples, blended_offsets, blended_normals, points, normals);
}

void Synthesizer::writeXYZ( QString filename, QVector<Vector3> &points, QVector<Vector3> &normals )
{
	QFile file(filename + ".xyz");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	for(int i = 0; i < (int) points.size(); i++)
	{
		Vec3d p = points[i];
		Vec3d n = normals[i];
		out << QString("%1 %2 %3 %4 %5 %6\n").arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]);
	}

	file.close();
}

void Synthesizer::saveSynthesisData( Structure::Node *node, QString prefix )
{
	if(!node || !node->property.contains("samples")) return;

	QVector<ParameterCoord> samples = node->property["samples"].value< QVector<ParameterCoord> >();
	QVector<double> offsets = node->property["offsets"].value< QVector<double> >();
	QVector<Vec2d> normals = node->property["normals"].value< QVector<Vec2d> >();

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
	QVector<double> offsets(num);
	QVector<Vec2d> normals(num);

	for(int i = 0; i < num; i++)
	{
		inF >> samples[i].u >> samples[i].v >> samples[i].theta >> samples[i].psi >> 
			offsets[i] >> normals[i][0] >> normals[i][1];
	}

	node->property["samples"].setValue(samples);
	node->property["offsets"].setValue(offsets);
	node->property["normals"].setValue(normals);
}
