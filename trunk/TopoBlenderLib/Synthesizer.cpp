#include "Synthesizer.h"

Synthesizer::Synthesizer()
{
}

void Synthesizer::generateRaysFromCurve( NURBSCurve & curve )
{
	// Clear
	result_rays.clear();
	result_t.clear();

	// Cylinder
	generateRaysWithinCylinder(curve);

	// End - 0
	generateRaysWithinHemiSphere( curve.GetNormal(0),  -curve.GetTangent(0) );
	result_t += QVector<double>(result_rays.size() - result_t.size(), 0);

	// End - 1
	generateRaysWithinHemiSphere( curve.GetNormal(1), curve.GetTangent(1) );
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

	sheet.GetFrame( 0, 1, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( -vDirection, uDirection, sheetNormal);

	sheet.GetFrame( 1, 0, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( vDirection, -uDirection, sheetNormal);

	sheet.GetFrame( 1, 1, sheetPoint, uDirection, vDirection, sheetNormal);
	generateRaysWithinSheetCorner( uDirection, vDirection, sheetNormal);
}


QVector<Vector3> Synthesizer::raysAroundAxis( Vector3 start, Vector3 axis, double range, int resolution )
{
	QVector<Vector3> rays;

	if ( range != (2 * M_PI) ) resolution += 1;

	double delta = range / resolution;

	for (int i = 0; i < resolution; i++)
	{
		Vector3 ray = ROTATED_VEC(start, delta * i, axis);

		rays.push_back(ray);
	}
}

void Synthesizer::generateRaysWithinCylinder( NURBSCurve curve )
{
	double delta_t = 1.0 / timeResolution;
	double delta_theta = 2 * M_PI / thetaResolution;

	// All cross sections 
	Vector3 current_position, curveTangent, biNormal, curveNormal;
	for (int i = 1; i < timeResolution; i++)
	{
		double curr_t = i * delta_t;
		curve.GetFrame(curr_t, current_position, curveTangent, curveNormal, biNormal);

		QVector<Vector3> rays = raysAroundAxis(curveNormal, curveTangent, 2 * M_PI, thetaResolution);
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
	for(int v_idx = 1; v_idx < vResolution; v_idx++)
	{
		for(int u_idx = 1; u_idx < uResolution; u_idx++)
		{
			u = u_idx * delta_u;
			v = v_idx * delta_v;

			sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

			// Up
			result_rays.push_back(sheetNormal);
			result_uv.push_back(std::make_pair(u, v));

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

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, -vDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Top
	v = 1;
	for(int u_idx = 1; u_idx < uResolution; u_idx++)
	{
		u = u_idx * delta_u;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, vDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Left
	u = 0;
	for(int v_idx = 1; v_idx < vResolution; v_idx++)
	{
		v = v_idx * delta_v;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, -uDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}

	// Right
	u = 1;
	for(int v_idx = 1; v_idx < vResolution; v_idx++)
	{
		v = v_idx * delta_v;
		sheet.GetFrame( u, v, sheetPoint, uDirection, vDirection, sheetNormal);

		QVector<Vector3> rays = raysAroundAxis(sheetNormal, uDirection, M_PI, thetaResolution);

		result_rays += rays;
		result_uv += QVector< std::pair<double, double> >(result_rays.size() - result_uv.size(), std::make_pair(u, v));
	}
}


void Synthesizer::generateRaysWithinSheetCorner( Vector3 X, Vector3 Y, Vector3 Z )
{
	// Assuming Z = cross(X, Y), but dot(X,Y) might not equal to 0

	Vector3 ZY = cross(Z, Y);
	double XY_range = 2 * M_PI - acos( dot(X, Y) );
	foreach (Vector3 v, raysAroundAxis(Z, ZY, M_PI, thetaResolution))
	{
		QVector<Vector3> rays = raysAroundAxis(v, Z, XY_range, phiResolution);

		result_rays += rays;
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

    assert(foundIntersection == true);

    return isetpoint;
}
