#include <QFile>
#include <QTextStream>

#include "NanoKdTree.h"
#include "SpherePackSampling.h"

#include "Synthesizer2.h"
#include "RMF.h"
#include "weld.h"

QVector<ParameterCoord> Synthesizer::genFeatureCoordsCurve( Structure::Curve * curve )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	
	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
	std::vector<Vec3d> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v]);

	// Auto resolution
	double resolution = curve->bbox().size().length() * 0.01;

	return genPointCoordsCurve(curve, meshPoints, resolution);
}

QVector<ParameterCoord> Synthesizer::genUniformCoordsCurve( Structure::Curve * curve, double resolution )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();

	// Auto resolution
	if(resolution < 0) resolution = curve->bbox().size().length() * 0.005;

	// Sample mesh surface
	std::vector<Vec3d> gp;
	static std::vector<Vec3d> samplePoints = SpherePackSampling::sample(model, 1e5, resolution, gp, 1);

	return genPointCoordsCurve(curve, samplePoints, resolution);
}

QVector<ParameterCoord> Synthesizer::genPointCoordsCurve( Structure::Curve * curve, const std::vector<Vec3d> & points, double resolution )
{
	QVector<ParameterCoord> samples;

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

		// Theta: angle from Z [0, PI]
		// Psi: angle from X on XY plane [0, 2*PI)
		double dotZ = dot(raydirection, Z);
		double theta = acos( dotZ );
		
		double dotX = dot(raydirection, X);
		double dotY = dot(raydirection, Y);
		Vec3d proj_ray = (dotX * X + dotY * Y).normalized();
		double psi = signedAngle(X, proj_ray, Z);

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

	// Auto resolution
	double resolution = sheet->bbox().size().length() * 0.01;

	return genPointCoordsSheet(sheet, meshPoints, resolution);
}

QVector<ParameterCoord> Synthesizer::genUniformCoordsSheet( Structure::Sheet * sheet, double resolution )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();

	// Auto resolution
	if(resolution < 0) resolution = curve->bbox().size().length() * 0.005;

	// Sample mesh surface
	std::vector<Vec3d> gp;
	static std::vector<Vec3d> samplePoints = SpherePackSampling::sample(model, 1e5, resolution, gp, 1);

	return genPointCoordsSheet(sheet, meshPoints, resolution);
}

QVector<ParameterCoord> Synthesizer::genPointCoordsSheet( Structure::Sheet * sheet, const std::vector<Vec3d> & points, double resolution /*= -1 */ )
{
	QVector<ParameterCoord> samples;

	Array2D_Vec4d sheetCoords = sheet->discretizedPoints(resolution).front();
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

	foreach(Vec3d point, points)
	{
		KDResults match;
		kdtree.k_closest(point, 1, match);
		int closest_idx = match.front().first;
		Vec4d c = allCoords[closest_idx];

		sheet->GetFrame( c[0], c[1], sheetPoint, X, vDirection, Z );
		Y = cross(Z, X);
	
		Vec3d raydirection = (point - sheetPoint).normalized();

		// Theta: angle from Z [0, PI]
		// Psi: angle from X on XY plane [0, 2*PI)
		double dotZ = dot(raydirection, Z);
		double theta = acos( dotZ );

		double dotX = dot(raydirection, X);
		double dotY = dot(raydirection, Y);
		Vec3d proj_ray = (dotX * X + dotY * Y).normalized();
		double psi = signedAngle(X, proj_ray, Z);

		samples.push_back(ParameterCoord(theta, psi, c[0], c[1]));
	}

	return samples;
}



void Synthesizer::computeOffsetsCurve( QVector<ParameterCoord> & samples, Structure::Curve * curve, QVector<double> &offsets, QVector<Vector3> &normals )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
	Octree octree(20, model);

	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	std::sort(samples.begin(), samples.end());
	foreach(ParameterCoord s, samples) samplePoints.push_back(curve->position(s.c));
	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();

	// Save to a file
	QFile file(QString("pointcloud_%1.xyz").arg(curve->id));
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	for(int r = 0; r < (int) samples.size(); r++)
	{
		ParameterCoord & sample = samples[r];

		int idx = xrefs[r];
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		Vec3d rayPos = curve->position(sample.c);
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);
		Vec3d vnormal = fnormals[SurfaceMeshModel::Face(faceIndex)];

		sample.offset = (isect-rayPos).norm();
		sample.fid = faceIndex;

		// Save position and normal
		out << QString("%1 %2 %3 ").arg(isect[0]).arg(isect[1]).arg(isect[2]) << 
			QString("%1 %2 %3").arg(vnormal[0]).arg(vnormal[1]).arg(vnormal[2]) << "\n";
	}

	file.close();
}


void Synthesizer::computeOffsetsSheet( QVector<ParameterCoord> & samples, Structure::Sheet * sheet, QVector<double> &offsets, QVector<Vector3> &normals )
{

}



void Synthesizer::synthesizeCurve( QVector<ParameterCoord> samples, Structure::Curve * curve, QString id )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");

	// Save to a file
	QFile file(QString("pointcloud_synth_%1_%2.xyz").arg(curve->id).arg(id));
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	std::sort(samples.begin(), samples.end());
	foreach(ParameterCoord s, samples) samplePoints.push_back(curve->position(s.c));
	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();

	for(int r = 0; r < (int) samples.size(); r++)
	{
		ParameterCoord sample = samples[r];

		int idx = xrefs[r];
		Vec3d X = rmf.U[idx].r.normalized();
		Vec3d Y = rmf.U[idx].s.normalized();
		Vec3d Z = rmf.U[idx].t.normalized();

		Vec3d rayPos = curve->position(sample.c);
		Vec3d rayDir = rotatedVec(Z, sample.theta, Y);
		rayDir = rotatedVec(rayDir, sample.psi, Z);

		Vec3d isect = rayPos + (rayDir.normalized() * sample.offset);
		Vec3d vnormal = fnormals[Surface_mesh::Face(sample.fid)];

		// Save position and normal
		out << QString("%1 %2 %3 ").arg(isect[0]).arg(isect[1]).arg(isect[2]) << 
			QString("%1 %2 %3").arg(vnormal[0]).arg(vnormal[1]).arg(vnormal[2]) << "\n";
	}

	file.close();
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
