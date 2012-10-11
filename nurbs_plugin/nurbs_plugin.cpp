#include "nurbs_plugin.h"

void nurbs_plugin::decorate()
{
    for(int i = 0; i < (int) nc.size(); i++)
    {
        NURBS::CurveDraw::draw( &nc[i] );
    }

    for(int i = 0; i < (int) rects.size(); i++)
    {
        NURBS::SurfaceDraw::draw ( &rects[i] );
    }
}

void nurbs_plugin::create()
{
	int degree = 3;

	// Curve example
	std::vector<Vec3d> cps;
	int steps = 10;
	double theta = M_PI * 5 / steps;
	double r = M_PI;

	for(int i = 0; i <= steps; i++)
	{
		double x = (double(i) / steps) * r;
		double y = sin(i * theta) * r * 0.25;

		cps.push_back(Vec3d(x - r * 0.5, y, cos(i*theta)));
	}

    std::vector<Scalar> weight(cps.size(), 1.0);

    nc.push_back(NURBSCurve(cps, weight, degree, false, true));

	// Rectangular surface
	double w = 1;
	double l = 2;

	int width = w * 5;
	int length = l * 5;

	std::vector< std::vector<Vec3d> > cpts( width, std::vector<Vec3d>(length, 1.0) );
	std::vector< std::vector<Scalar> > weights( width, std::vector<Scalar>(length, 1.0) );

	double omega = M_PI * 3 / qMin(width, length);

	Vec2d surface_pos(0,1);

	for(int i = 0; i < width; i++)
		for(int j = 0; j < length; j++)
		{
			double x = double(i) / width;
			double y = double(j) / length;

			double delta = sqrt(x*x + y*y) * 0.5;

			cpts[i][j] = Vec3d(surface_pos.x() + x * w, surface_pos.y() + y * l, delta + sin(j * omega) * qMin(width, length) * 0.02);
		}

    rects.push_back( NURBSRectangle(cpts, weights, degree, degree, false, false, true, true) );

    // Set scene bounds
    QBox3D bbox;
    foreach(Vector3 v, cps) bbox.unite(v);
    foreach(std::vector<Vec3d> vs, cpts) foreach(Vector3 v, vs) bbox.unite(v);
    Vector3 a = bbox.minimum();
    Vector3 b = bbox.maximum();
    drawArea()->setSceneBoundingBox(qglviewer::Vec(a.x(), a.y(), a.z()), qglviewer::Vec(b.x(), b.y(), b.z()));
}

Q_EXPORT_PLUGIN (nurbs_plugin)
