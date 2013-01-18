#include <omp.h>

#include <QFile>
#include <QTextStream>

#include "NanoKdTree.h"
#include "SpherePackSampling.h"

#include "Synthesizer.h"
#include "RMF.h"
#include "weld.h"

typedef std::pair<ParameterCoord,int> ParameterCoordInt;
bool comparatorParameterCoordInt ( const ParameterCoordInt& l, const ParameterCoordInt& r)
{ return l.first < r.first; }

#define CURVE_FRAME_RESOLUTION 0.01
#define SHEET_FRAME_RESOLUTION 0.01

// Sampling
#define RANDOM_COUNT 1e4
#define UNIFORM_RESOLUTION 0.01
#define EDGE_RESOLUTION 0.05

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

void Synthesizer::localSphericalToGlobal( Vector3 X, Vector3 Y, Vector3 Z, double theta, double psi, Vector3 &v )
{
	Q_UNUSED(X);
	v = rotatedVec(Z, theta, Y);
	v = rotatedVec(v, psi, Z);
}

void Synthesizer::globalToLocalSpherical( Vector3 X, Vector3 Y, Vector3 Z, double &theta, double &psi, Vector3 v )
{
	// Theta: angle from Z [0, PI]
	// Psi: angle from X on XY plane [0, 2*PI)
	double dotZ = dot(v, Z);
	theta = acos( dotZ );

	double dotX = dot(v, X);
	double dotY = dot(v, Y);
	Vec3d proj_v = (dotX * X + dotY * Y).normalized();
	psi = signedAngle(X, proj_v, Z);
}

QVector<ParameterCoord> Synthesizer::genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points )
{
	QVector<ParameterCoord> samples(points.size());

	// Auto resolution
	double resolution = curve->bbox().size().length() * CURVE_FRAME_RESOLUTION;

	Array1D_Vec4d curveCoords = curve->discretizedPoints(resolution).front();

	qDebug() << "Curve resolution count = " << curveCoords.size();

	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	foreach(Vec4d c, curveCoords) samplePoints.push_back(curve->position(c));
	RMF rmf(samplePoints);
	rmf.compute();

	// Add all curve points to kd-tree
	NanoKdTree kdtree;
	foreach(Vec4d c, curveCoords) kdtree.addPoint( curve->position(c) );
	kdtree.build();

	// Project
	NURBSCurve mycurve;
	#pragma omp parallel for private(mycurve)
	for(int i = 0; i < (int)points.size(); i++)
	{
		mycurve = curve->curve;

		double theta, psi;

		Vec3d point = points[i];

		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = curveCoords[closest_idx];

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

	NURBSRectangle r;

	#pragma omp parallel for private(r)
	for(int i = 0; i < (int)points.size(); i++)
	{
		r = sheet->surface;

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
	SurfaceMeshModel * model = node->property["mesh"].value<SurfaceMeshModel*>();

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
	SurfaceMeshModel * model = node->property["mesh"].value<SurfaceMeshModel*>();
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

QVector<ParameterCoord> Synthesizer::genRandomCoords(Node *node, double sampling_resolution)
{
	SurfaceMeshModel * model = node->property["mesh"].value<SurfaceMeshModel*>();

	// Sample mesh surface
	int samples_count = RANDOM_COUNT;
	std::vector<Vec3d> samplePoints;
	samplePoints = SpherePackSampling::getRandomSamples(model, samples_count);

	if(node->type() == Structure::CURVE)
		return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
	else
		return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genUniformCoords(Node *node, double sampling_resolution)
{
    SurfaceMeshModel * model = node->property["mesh"].value<SurfaceMeshModel*>();

    // Auto resolution
    if(sampling_resolution < 0) sampling_resolution = node->bbox().size().length() * UNIFORM_RESOLUTION;

    // Sample mesh surface
    std::vector<Vec3d> samplePoints, gp;
    samplePoints = SpherePackSampling::sample(model, 1e5, sampling_resolution, gp, 1);

    if(node->type() == Structure::CURVE)
        return genPointCoordsCurve((Structure::Curve*)node, samplePoints);
    else
        return genPointCoordsSheet((Structure::Sheet*)node, samplePoints);
}

Vec3d Synthesizer::intersectionPoint( const Ray & ray, const Octree * useTree, int * faceIndex )
{
	HitResult res;
	Vec3d isetpoint(0);

	double minDistance = DBL_MAX;

	foreach( int i, useTree->intersectRay( ray, 0.1, false ) )
	{
		useTree->intersectionTestOld(SurfaceMeshModel::Face(i), ray, res);

		// find the nearest intersection point
		if(res.hit)
		{
			if (res.distance < minDistance)
			{
				minDistance = res.distance;
				isetpoint = ray.origin + (ray.direction * res.distance);
				if(faceIndex) *faceIndex = i;
			}
			return isetpoint;
		}
	}

	assert(false);

	return isetpoint;
}

void Synthesizer::sampleGeometryCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QVector<double> &offsets, QVector<Vec2d> &normals )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
	Octree octree(20, model);

	offsets.clear();
	offsets.resize(samples.size());

	normals.clear();
	normals.resize(samples.size());

	// Generate consistent frames along curve
	QVector<int> oldIndices;
	sortSamplesCurve(samples, oldIndices);

	// Weld for RMF
	std::vector<Vec3d> samplePoints;
	foreach(ParameterCoord s, samples) samplePoints.push_back( curve->position(Vec4d(s.u)) );
	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();

	ParameterCoord * samplesArray = samples.data();

	qDebug() << "Curve RMF count = " << rmf.U.size() << ", Samples = " << samples.size();

	NURBSCurve mycurve;
	#pragma omp parallel for private(mycurve)
	for(int i = 0; i < (int)samples.size(); i++)
	{
		mycurve = curve->curve;

		ParameterCoord sample = samplesArray[i];
		
		int idx = xrefs[i];
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		Vec4d coordinate(sample.u);
		Vec3d rayPos = mycurve.GetPosition( coordinate[0] );
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);

		// Store the offset
		offsets[ oldIndices[i] ] = (isect - rayPos).norm();

		// Code the normal relative to local frame
		Vec2d normalCoord(0);
		Vec3d v = fnormals[ SurfaceMeshModel::Face(faceIndex) ];

		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], v);

		normals[ oldIndices[i] ] = normalCoord;
	}
}

void Synthesizer::sampleGeometrySheet( QVector<ParameterCoord> samples, Structure::Sheet * sheet, QVector<double> &offsets, QVector<Vec2d> &normals )
{
	SurfaceMeshModel * model = sheet->property["mesh"].value<SurfaceMeshModel*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
	Octree octree(20, model);

	offsets.clear();
	offsets.resize( samples.size() );

	normals.clear();
	normals.resize( samples.size() );

	qDebug() << "Sheet samples = " << samples.size();

	ParameterCoord * samplesArray = samples.data();

	#pragma omp parallel for
	for(int i = 0; i < (int)samples.size(); i++)
	{
		ParameterCoord sample = samplesArray[i];

		Vector3 X(0), Y(0), Z(0);
		Vector3 vDirection(0), rayPos(0);

		NURBSRectangle r = sheet->surface;

		r.GetFrame( sample.u, sample.v, rayPos, X, vDirection, Z );
		Y = cross(Z, X);

		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		// Store the offset
		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);
		offsets[i] = (isect - rayPos).norm();

		// Code the normal relative to local frame
		Vec2d normalCoord;
		Vec3d v = fnormals[SurfaceMeshModel::Face(faceIndex)];

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
	QVector<int> oldIndices;
	sortSamplesCurve(in_samples, oldIndices);

	std::vector<Vec3d> samplePoints;
	foreach(ParameterCoord s, in_samples) samplePoints.push_back(base_curve->position(Vec4d(s.u)));

	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();

	for(int i = 0; i < (int) in_samples.size(); i++)
	{
		ParameterCoord sample = in_samples[i];

		Vec3d rayPos, rayDir;

		int idx = xrefs[i];
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		rayPos = base_curve->position(Vec4d(sample.u));

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);

		// Reconstructed point
		Vec3d isect = rayPos + (rayDir.normalized() * in_offsets[ oldIndices[i] ]);
		out_points[ oldIndices[i] ] = isect;

		// Reconstructed normal
		Vec2d normalCoord = in_normals[ oldIndices[i] ];
		Vec3d normal;
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);
		out_normals[ oldIndices[i] ] = normal.normalized();
	}
}

void Synthesizer::reconstructGeometrySheet( Structure::Sheet * base_sheet,  QVector<ParameterCoord> in_samples, QVector<double> &in_offsets,
	QVector<Vec2d> &in_normals, QVector<Vector3> &out_points, QVector<Vector3> &out_normals )
{
	out_points.clear();
	out_points.resize(in_samples.size());

	out_normals.clear();
	out_normals.resize(in_samples.size());

	for(int i = 0; i < (int) in_samples.size(); i++)
	{
		Vector3 X, Y, Z;
		Vector3 vDirection, sheetPoint;

		Vec3d rayPos, rayDir;

		ParameterCoord sample = in_samples[i];

		base_sheet->surface.GetFrame( sample.u, sample.v, sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);

		rayPos = base_sheet->position(Vec4d(sample.u, sample.v, 0, 0));

		localSphericalToGlobal(X, Y, Z, sample.theta, sample.psi, rayDir);

		// Reconstructed point
		Vec3d isect = rayPos + (rayDir.normalized() * in_offsets[i]);
		out_points[i] = isect;

		// Reconstructed normal
		Vec2d normalCoord = in_normals[i];
		Vec3d normal;
		localSphericalToGlobal(X, Y, Z, normalCoord[0], normalCoord[1], normal);
		out_normals[i] = normal.normalized();
	}
}

void Synthesizer::prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2, int s )
{
	if(!curve1 || !curve2 || !curve1->property.contains("mesh") || !curve2->property.contains("mesh")) return;

	QElapsedTimer timer; timer.start();
	qDebug() << "Generating samples..";

	// Sample two curves
	QVector<ParameterCoord> samples;

	if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
	if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

	if (s & Synthesizer::Features)	samples += genFeatureCoords(curve1) + genFeatureCoords(curve2);
	if (s & Synthesizer::Edges)	samples += genEdgeCoords(curve1) + genEdgeCoords(curve2);
	if (s & Synthesizer::Random)	samples += genRandomCoords(curve1) + genRandomCoords(curve2);
	if (s & Synthesizer::Uniform)	samples += genUniformCoords(curve1) + genUniformCoords(curve2);

	qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());timer.restart();

	qDebug() << "Re-sampling mesh..";

	// Re-sample the meshes
	QVector<double> offsets1, offsets2;
	QVector<Vec2d> normals1, normals2;
	sampleGeometryCurve(samples, curve1, offsets1, normals1);
	sampleGeometryCurve(samples, curve2, offsets2, normals2);

	curve1->property["samples"].setValue(samples);
	curve1->property["offsets"].setValue(offsets1);
	curve1->property["normals"].setValue(normals1);

	curve2->property["samples"].setValue(samples);
	curve2->property["offsets"].setValue(offsets2);
	curve2->property["normals"].setValue(normals2);

	qDebug() << QString("Resampling Time [ %1 ms ]\n==\n").arg(timer.elapsed());
}

void Synthesizer::prepareSynthesizeSheet( Structure::Sheet * sheet1, Structure::Sheet * sheet2, int s )
{
	if(!sheet1 || !sheet2 || !sheet1->property.contains("mesh") || !sheet2->property.contains("mesh")) return;

    QElapsedTimer timer; timer.start();
    qDebug() << "Generating samples..";

	// Sample two sheets
	QVector<ParameterCoord> samples;

	if (s & Synthesizer::All)	s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random | Synthesizer::Uniform;
	if (s & Synthesizer::AllNonUniform) s = Synthesizer::Features | Synthesizer::Edges | Synthesizer::Random;

	if (s & Synthesizer::Features)	samples += genFeatureCoords(sheet1) + genFeatureCoords(sheet2);
	if (s & Synthesizer::Edges)	samples += genEdgeCoords(sheet1) + genEdgeCoords(sheet2);
	if (s & Synthesizer::Random)	samples += genRandomCoords(sheet1) + genRandomCoords(sheet2);
	if (s & Synthesizer::Uniform)	samples += genUniformCoords(sheet1) + genUniformCoords(sheet2);

    qDebug() << QString("Samples Time [ %1 ms ]").arg(timer.elapsed());

	// Re-sample the meshes
	QVector<double> offsets1, offsets2;
	QVector<Vec2d> normals1, normals2;
	sampleGeometrySheet(samples, sheet1, offsets1, normals1);
	sampleGeometrySheet(samples, sheet2, offsets2, normals2);

	sheet1->property["samples"].setValue(samples);
	sheet1->property["offsets"].setValue(offsets1);
	sheet1->property["normals"].setValue(normals1);

	sheet2->property["samples"].setValue(samples);
	sheet2->property["offsets"].setValue(offsets2);
	sheet2->property["normals"].setValue(normals2);
}


void Synthesizer::blendGeometryCurves( Structure::Curve * curve1, Structure::Curve * curve2, double alpha, QVector<Vector3> &points, QVector<Vector3> &normals )
{
	if(!curve1->property.contains("samples") || !curve2->property.contains("samples")) return;

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
	QFile file(filename);
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

void Synthesizer::saveSynthesisData( Structure::Node *node )
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

	QFile file(node->id + ".txt");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	out <<  samples.size() << "\n";

	for(int i = 0; i < (int) samples.size(); i++)
	{
		out << samples[i].u << "\t" << samples[i].v<< "\t" << samples[i].theta<< "\t" << samples[i].psi<< "\t"
			<< offsets[i]<< "\t" << normals[i][0]<< "\t" << normals[i][1] << "\n";
	}

	file.close();
}

void Synthesizer::loadSynthesisData( Structure::Node *node )
{
	if(!node) return;

	QFile file(node->id + ".txt");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QTextStream inF(&file);

	int num;
	inF >> num;

	QVector<ParameterCoord> samples(num);
	QVector<double> offsets(num);
	QVector<Vec2d> normals(num);

	for(int i = 0; i < num; i++)
	{
		inF >> samples[i].u >> samples[i].v >> samples[i].theta >> samples[i].psi
			>> offsets[i] >> normals[i][0] >> normals[i][1];
	}

	node->property["samples"].setValue(samples);
	node->property["offsets"].setValue(offsets);
	node->property["normals"].setValue(normals);
}
