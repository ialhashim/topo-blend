#include "Synthesizer.h"

Synthesizer::Synthesizer()
{
	timeResolution = 20;
	uResolution = vResolution = 20;
	thetaResolution = phiResolution = 10;
}

void Synthesizer::generateRaysFromCurve( NURBSCurve & curve )
{
	// Clear
	result_rays.clear();
	result_t.clear();


	// Generate consistent frames along curve
	std::vector<Vec3d> samplePoints;
	for(int i = 0; i <= timeResolution; i++) samplePoints.push_back(curve.GetPosition(double(i) / timeResolution));
	RMF rmf(samplePoints);
	rmf.compute();
	frames = rmf.U;

	// Cylinder
	numV.push_back(result_rays.size());
	generateRaysWithinCylinder(curve);

	// End - 0
	numV.push_back(result_rays.size());
	generateRaysWithinHemiSphere( frames.front().s,  -frames.front().t );
	result_t += QVector<double>(result_rays.size() - result_t.size(), 0);

	// End - 1
	numV.push_back(result_rays.size());
	generateRaysWithinHemiSphere( frames.back().s,  frames.back().t );
	result_t += QVector<double>(result_rays.size() - result_t.size(), 1);
}

void Synthesizer::generateRaysFromSheet( NURBSRectangle &sheet )
{
	// Clear
	result_rays.clear();
	result_uv.clear();

	/// The up and down planes
	generateRaysOnTwoSidedPlane(sheet);

	/// Four edges
	generateRaysOnEdges(sheet);

	/// Four Corners
	Vector3 sheetNormal, uDirection, vDirection, sheetPoint;
	sheet.GetFrame( 0, 0, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( uDirection, vDirection, sheetNormal);
	result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(0, 0));

	sheet.GetFrame( 0, 1, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( -vDirection, uDirection, sheetNormal);
	result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(0, 1));

	sheet.GetFrame( 1, 0, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( vDirection, -uDirection, sheetNormal);
	result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(1, 0));

	sheet.GetFrame( 1, 1, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( -uDirection, -vDirection, sheetNormal);
	result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(1, 1));
}

void Synthesizer::generateRaysWithinCylinder( NURBSCurve curve )
{
	double delta_t = 1.0 / timeResolution;

	// All cross sections 
	Vector3 current_position, curveTangent, biNormal, curveNormal;
	for (int i = 1; i < timeResolution; i++)
	{
		double curr_t = i * delta_t;
		RMF::Frame frame = frames[i];

		QVector<Vector3> rays = raysAroundAxis(frame.r, frame.t, 2 * M_PI, thetaResolution);
		QVector<double> ts(rays.size(), curr_t);

		result_rays += rays;
		result_t += ts;
	}
}

void Synthesizer::generateRaysWithinHemiSphere( Vector3 X, Vector3 Z )
{
	foreach (Vector3 v, raysAroundAxis(Z, X, 0.5 * M_PI, thetaResolution))
	{
		QVector<Vector3> ringRays = raysAroundAxis(v, Z, 2 * M_PI, phiResolution);

		result_rays += ringRays;
	}
}

void Synthesizer::generateRaysOnTwoSidedPlane( NURBSRectangle &sheet )
{
	double delta_u = 1.0 / uResolution;
	double delta_v = 1.0 / vResolution;

	// Leave one for the edge
	double u, v;
	Vector3 sheetNormal, uDirection, vDirection, sheetPoint;

	for(int v_idx = 1; v_idx < vResolution; v_idx++){
		for(int u_idx = 1; u_idx < uResolution; u_idx++){
			u = u_idx * delta_u;
			v = v_idx * delta_v;

			sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

			// Up
			result_rays.push_back(sheetNormal);
			result_uv.push_back(std::make_pair(u, v));
		}
	}

	for(int v_idx = 1; v_idx < vResolution; v_idx++){
		for(int u_idx = 1; u_idx < uResolution; u_idx++){
			u = u_idx * delta_u;
			v = v_idx * delta_v;

			sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

			// Down
			result_rays.push_back(-sheetNormal);
			result_uv.push_back(std::make_pair(u, v));
		}
	}
}

void Synthesizer::generateRaysOnEdges( NURBSRectangle &sheet )
{
	// Temp data
	double u, v;
	Vector3 sheetNormal, uDirection, vDirection, sheetPoint;


	/// Four edges
	double delta_u = 1.0 / uResolution;
	double delta_v = 1.0 / vResolution;

	// Bottom
	v = 0;
	for(int u_idx = 1; u_idx < uResolution; u_idx++)
	{
		u = u_idx * delta_u;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, uDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Top
	v = 1;
	for(int u_idx = 1; u_idx < uResolution; u_idx++)
	{
		u = u_idx * delta_u;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, -uDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Left
	u = 0;
	for(int v_idx = 1; v_idx < vResolution; v_idx++)
	{
		v = v_idx * delta_v;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, -vDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Right
	u = 1;
	for(int v_idx = 1; v_idx < vResolution; v_idx++)
	{
		v = v_idx * delta_v;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, vDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}
}

void Synthesizer::generateRaysWithinSheetCorner( Vector3 X, Vector3 Y, Vector3 Z )
{
	// Assuming Z = cross(X, Y), but dot(X,Y) might not equal to 0

	Vector3 Z_X = cross(Z, -X);
	double XY_range = acos( dot(X, Y) );
	foreach (Vector3 v, raysAroundAxis(Z, Z_X, M_PI, thetaResolution))
	{
		QVector<Vector3> rays = raysAroundAxis(v, Z, XY_range, phiResolution);

		result_rays += rays;
	}
}

QVector<Vector3> Synthesizer::raysAroundAxis( Vector3 start, Vector3 axis, double range, int resolution )
{
	QVector<Vector3> rays;
	double delta = range / resolution;

	if ( range < (2 * M_PI) ) resolution += 1;

	for (int i = 0; i < resolution; i++)
	{
		Vector3 ray = ROTATED_VEC(start, delta * i, axis);

		rays.push_back(ray);
	}

	return rays;
}

void Synthesizer::resampleCurve( Structure::Curve * curve )
{
	SurfaceMeshModel * remeshed = new SurfaceMeshModel(curve->id + "_remeshed.off", curve->id);
	SurfaceMeshModel * mesh = curve->property["mesh"].value<SurfaceMeshModel *>();
	Octree tree(50, mesh);

	generateRaysFromCurve(curve->curve);

	for(int i = 0; i < result_t.size(); i++)
	{
		double t = result_t[i];
		Vec3d rayDir = result_rays[i];
		Vec3d pos = curve->position(Vec4d(t));

		isect_points.push_back( intersectionPoint(Ray(pos,rayDir), &tree) );
	}
}

void Synthesizer::resampleSheet( Structure::Sheet * sheet )
{
	SurfaceMeshModel * remeshed = new SurfaceMeshModel(sheet->id + "_remeshed.off", sheet->id);
	SurfaceMeshModel * mesh = sheet->property["mesh"].value<SurfaceMeshModel *>();
	Octree tree(50, mesh);

	generateRaysFromSheet(sheet->surface);

	for(int i = 0; i < result_uv.size(); i++)
	{
		std::pair<double,double> uv = result_uv[i];
		Vec3d rayDir = result_rays[i];
		Vec3d pos = sheet->position(Vec4d(uv.first,uv.second,0,0));

		isect_points.push_back( intersectionPoint(Ray(pos,rayDir), &tree) );
	}
}

Vec3d Synthesizer::intersectionPoint( Ray ray, Octree * useTree, int * faceIndex )
{
	HitResult res;
	Vec3d isetpoint(0);

	QSet<int> results = useTree->intersectRay( ray, ray.thickness, false );

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
		}
	}

	//assert(foundIntersection == true);

	return isetpoint;
}


void Synthesizer::buildFacesCurve()
{
	buildFacesCylinder();

	buildFacesHemiSphere();

	buildFacesHemiSphere();
}


void Synthesizer::buildFacesCylinder()
{
	int v0, v1, v2, v3;

	for(int v = 0; v < timeResolution - 1; v++){
		for(int u = 0; u < thetaResolution; u++){

			int offset = (thetaResolution * (v));

			v0 = u + offset;
			v1 = ((u + 1) % thetaResolution) + offset;
			v2 = v1 + thetaResolution;
			v3 = v0 + thetaResolution;

			QVector<int> f;
			f.push_back(v0);
			f.push_back(v1);
			f.push_back(v2);
			f.push_back(v3);

			faces.push_back(f);
		}
	}
}

void Synthesizer::buildFacesHemiSphere()
{

}
