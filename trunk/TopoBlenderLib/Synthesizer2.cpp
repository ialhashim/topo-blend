#include <QFile>
#include <QTextStream>

#include "NanoKdTree.h"
#include "SpherePackSampling.h"

#include "Synthesizer2.h"
#include "RMF.h"
#include "weld.h"

QVector<SynthSample> Synthesizer2::generateFeatureSamplesCurve( Structure::Curve * curve )
{
	QVector<SynthSample> samples;
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	
	// Collect original mesh points
	Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
	std::vector<Vec3d> meshPoints;
	foreach(Vertex v, model->vertices()) meshPoints.push_back(points[v]);

	// Auto resolution
	double resolution = curve->bbox().size().length() * 0.01;

	return generateSamplesCurve(curve, meshPoints, resolution);
}

QVector<SynthSample> Synthesizer2::generateUniformSamplesCurve( Structure::Curve * curve, double resolution )
{
	QVector<SynthSample> samples;
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();

	// Auto resolution
	if(resolution < 0) resolution = curve->bbox().size().length() * 0.01;

	// Sample mesh surface
	std::vector<Vec3d> gp;
	static std::vector<Vec3d> samplePoints = SpherePackSampling::sample(model, 1e5, resolution, gp, 1);

	return generateSamplesCurve(curve, samplePoints, resolution);
}

QVector<SynthSample> Synthesizer2::generateSamplesCurve( Structure::Curve * curve, const std::vector<Vec3d> & points, double resolution )
{
	QVector<SynthSample> samples;

	Array1D_Vec4d curveCoords = curve->discretizedPoints(resolution * 0.5).front();

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

		Vec3d tangent = rmf.U[closest_idx].t.normalized();
		Vec3d normal = rmf.U[closest_idx].r.normalized();
		Vec3d binormal = rmf.U[closest_idx].s.normalized();

		Vec3d curvePoint = curve->position(c);
		Vec3d raydirection = (point - curvePoint).normalized();

		double dist = dot(normal, raydirection);
		Vec3d projectedDir = raydirection - (dist * normal);
		double angle = signedAngle(tangent, projectedDir, normal);

		samples.push_back(SynthSample(c, angle, dist));
	}

	return samples;
}

QVector<SynthSample> Synthesizer2::generateSamplesSheet( Structure::Sheet * sheet )
{
	QVector<SynthSample> samples;

	return samples;
}

QVector<SynthSample> Synthesizer2::generateUniformSamplesSheet( Structure::Sheet * sheet, double resolution )
{
	QVector<SynthSample> samples;

	return samples;
}

void Synthesizer2::outputSamplesCurve( QVector<SynthSample> & samples, Structure::Curve * curve )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();
	model->update_face_normals();
	Vector3FaceProperty fnormals = model->face_property<Vec3d>("f:normal");
	Octree octree(20, model);

	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	std::sort(samples.begin(), samples.end());
	foreach(SynthSample s, samples) samplePoints.push_back(curve->position(s.c));
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
		SynthSample & sample = samples[r];

		int idx = xrefs[r];
		Vec3d tangent = rmf.U[idx].t;
		Vec3d normal = rmf.U[idx].r;
		Vec3d binormal = rmf.U[idx].s;

		Vec3d rayPos = curve->position(sample.c);
		Vec3d rayDir = rotatedVec(tangent, sample.signedAngle, normal) + (sample.signedDistance * normal);

		Ray ray( rayPos, rayDir );
		int faceIndex = 0;

		Vec3d isect = intersectionPoint(ray, &octree, &faceIndex);
		Vec3d vnormal = fnormals[SurfaceMeshModel::Face(faceIndex)];

		sample.r = (isect-rayPos).norm();

		// Save position and normal
		out << QString("%1 %2 %3 ").arg(isect[0]).arg(isect[1]).arg(isect[2]) << 
			QString("%1 %2 %3").arg(vnormal[0]).arg(vnormal[1]).arg(vnormal[2]) << "\n";
	}

	file.close();
}

void Synthesizer2::synthesizeCurve( QVector<SynthSample> samples, Structure::Curve * curve, QString id )
{
	SurfaceMeshModel * model = curve->property["mesh"].value<SurfaceMeshModel*>();

	// Save to a file
	QFile file(QString("pointcloud_synth_%1_%2.xyz").arg(curve->id).arg(id));
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	std::sort(samples.begin(), samples.end());
	foreach(SynthSample s, samples) samplePoints.push_back(curve->position(s.c));
	std::vector<size_t> xrefs;
	weld(samplePoints, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	RMF rmf(samplePoints);
	rmf.compute();

	for(int r = 0; r < (int) samples.size(); r++)
	{
		SynthSample sample = samples[r];

		int idx = xrefs[r];
		Vec3d tangent = rmf.U[idx].t;
		Vec3d normal = rmf.U[idx].r;
		Vec3d binormal = rmf.U[idx].s;

		Vec3d rayPos = curve->position(sample.c);
		Vec3d rayDir = rotatedVec(tangent, sample.signedAngle, normal) + (sample.signedDistance * normal);

		Vec3d isect = rayPos + (rayDir.normalized() * sample.r);
		Vec3d vnormal = rayDir;

		// Save position and normal
		out << QString("%1 %2 %3 ").arg(isect[0]).arg(isect[1]).arg(isect[2]) << 
			QString("%1 %2 %3").arg(vnormal[0]).arg(vnormal[1]).arg(vnormal[2]) << "\n";
	}

	file.close();
}

Vec3d Synthesizer2::intersectionPoint( Ray ray, Octree * useTree, int * faceIndex )
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
