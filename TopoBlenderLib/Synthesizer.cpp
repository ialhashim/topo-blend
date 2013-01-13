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

#define UNIFORM_RESOLUTION 0.01
#define CURVE_FRAME_RESOLUTION 0.0001
#define SHEET_FRAME_RESOLUTION 0.01

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



QVector<ParameterCoord> Synthesizer::genFeatureCoordsCurve( Structure::Curve * curve )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	
	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
	std::vector<Vec3d> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v]);

	return genPointCoordsCurve(curve, meshPoints);
}

QVector<ParameterCoord> Synthesizer::genUniformCoordsCurve( Structure::Curve * curve, double sampling_resolution )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();

	// Auto resolution
	if(sampling_resolution < 0) sampling_resolution = curve->bbox().size().length() * UNIFORM_RESOLUTION;

	// Sample mesh surface
	std::vector<Vec3d> gp;
	static std::vector<Vec3d> samplePoints = SpherePackSampling::sample(model, 1e5, sampling_resolution, gp, 1);

	return genPointCoordsCurve(curve, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points )
{
	QVector<ParameterCoord> samples;

	// Auto resolution
	double resolution = curve->bbox().size().length() * CURVE_FRAME_RESOLUTION;

	Array1D_Vec4d curveCoords = curve->discretizedPoints(resolution).front();

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
	double theta, psi;
	foreach(Vec3d point, points)
	{
		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = curveCoords[closest_idx];

		Vec3d X = rmf.U[closest_idx].r.normalized();
		Vec3d Y = rmf.U[closest_idx].s.normalized();
		Vec3d Z = rmf.U[closest_idx].t.normalized();

		Vec3d curvePoint = curve->position(c);
		Vec3d raydirection = (point - curvePoint).normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);
		
		samples.push_back(ParameterCoord(theta, psi, c[0]));
	}

	return samples;
}


QVector<ParameterCoord> Synthesizer::genFeatureCoordsSheet( Structure::Sheet * sheet )
{
	QVector<ParameterCoord> samples;
	SurfaceMeshModel * model = sheet->property["mesh"].value<SurfaceMeshModel*>();

	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
	std::vector<Vec3d> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v]);

	return genPointCoordsSheet(sheet, meshPoints);
}

QVector<ParameterCoord> Synthesizer::genUniformCoordsSheet( Structure::Sheet * sheet, double resolution )
{
	SurfaceMeshModel * model = sheet->property["mesh"].value<SurfaceMeshModel*>();

	// Auto resolution
	if(resolution < 0) resolution = sheet->bbox().size().length() * UNIFORM_RESOLUTION;

	// Sample mesh surface
	std::vector<Vec3d> gp;
	static std::vector<Vec3d> samplePoints = SpherePackSampling::sample(model, 1e5, resolution, gp, 1);

	return genPointCoordsSheet(sheet, samplePoints);
}

QVector<ParameterCoord> Synthesizer::genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vec3d> & points )
{
	QVector<ParameterCoord> samples;

	// Auto resolution
	double resolution = sheet->bbox().size().length() * SHEET_FRAME_RESOLUTION;

	Array2D_Vec4d sheetCoords = sheet->discretizedPoints(resolution);
	Array1D_Vec4d allCoords;
	
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

	// Project
	Vector3 X, Y, Z;
	Vector3 vDirection, sheetPoint;

	double theta, psi;
	foreach(Vec3d point, points)
	{
		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = allCoords[closest_idx];

		sheet->surface.GetFrame( c[0], c[1], sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);
	
		Vec3d raydirection = (point - sheetPoint).normalized();

		globalToLocalSpherical(X, Y, Z, theta, psi, raydirection);

		samples.push_back(ParameterCoord(theta, psi, c[0], c[1]));
	}

	return samples;
}


Vec3d Synthesizer::intersectionPoint( Ray ray, Octree * useTree, int * faceIndex )
{
	HitResult res;
	Vec3d isetpoint(0);

	QSet<int> results = useTree->intersectRay( ray, 0.1, false );

	double minDistance = DBL_MAX;
	bool foundIntersection;

	foreach(int i, results)
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
			foundIntersection = true;
			break;
		}
	}

	assert(foundIntersection == true);

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

	std::vector<Vec3d> samplePoints;
	foreach(ParameterCoord s, samples) samplePoints.push_back(curve->position(Vec4d(s.u)));

	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();
	
	for(int i = 0; i < (int) samples.size(); i++)
	{
		ParameterCoord & sample = samples[i];

		int idx = xrefs[i];
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		Vec3d rayPos = curve->position(Vec4d(sample.u));
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);

		// Store the offset
		offsets[ oldIndices[i] ] = (isect - rayPos).norm();

		// Code the normal relative to local frame
		Vec2d normalCoord;
		Vec3d v = fnormals[SurfaceMeshModel::Face(faceIndex)];

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

	Vector3 X, Y, Z;
	Vector3 vDirection, sheetPoint;

	for(int i = 0; i < (int) samples.size(); i++)
	{
		ParameterCoord & sample = samples[i];

		sheet->surface.GetFrame( sample.u, sample.v, sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);

		Vec3d rayPos = sheet->position( Vec4d(sample.u, sample.v, 0, 0) );
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;


		// Store the offset
		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);
		offsets.push_back( (isect-rayPos).norm() );

		// Code the normal relative to local frame
		Vec2d normalCoord;
		Vec3d v = fnormals[SurfaceMeshModel::Face(faceIndex)];

		globalToLocalSpherical(X, Y, Z, normalCoord[0], normalCoord[1], v);

		normals.push_back( normalCoord );
	}
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

	Vec3d rayPos, rayDir;
	for(int i = 0; i < (int) in_samples.size(); i++)
	{
		ParameterCoord sample = in_samples[i];

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

	Vector3 X, Y, Z;
	Vector3 vDirection, sheetPoint;

	Vec3d rayPos, rayDir;
	for(int i = 0; i < (int) in_samples.size(); i++)
	{
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


void Synthesizer::prepareSynthesizeCurve( Structure::Curve * curve1, Structure::Curve * curve2 )
{
	if(!curve1->property.contains("mesh") || !curve2->property.contains("mesh")) return;

	// Sample two curves
	QVector<ParameterCoord> samples = genFeatureCoordsCurve(curve1) + genFeatureCoordsCurve(curve2) 
									+ genUniformCoordsCurve(curve1) + genUniformCoordsCurve(curve2);

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
}

void Synthesizer::prepareSynthesizeSheet( Structure::Sheet * sheet1, Structure::Sheet * sheet2 )
{
	if(!sheet1->property.contains("mesh") || !sheet2->property.contains("mesh")) return;

	// Sample two sheets
	QVector<ParameterCoord> samples = genFeatureCoordsSheet(sheet1) + genFeatureCoordsSheet(sheet2) 
									+ genUniformCoordsSheet(sheet1) + genUniformCoordsSheet(sheet2);

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

