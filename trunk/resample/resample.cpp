#include "resample.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "SpherePackSampling.h"
#include "../segment/CustomDrawObjects.h"

void myresample::createEdit()
{
    rw = new ResampleWidget(this);
    rw->show();
}

void myresample::doResample()
{
    drawArea()->deleteAllRenderObjects();

    int millions = 1e5;
    double s = 0.01;

    PointSoup * ps = new PointSoup;

    std::vector<Vec3d> samples = SpherePackSampling::sample(mesh(), millions, mesh()->bbox().size().length() * s);

    foreach(Vec3d p, samples)
        ps->addPoint(p);

    drawArea()->addRenderObject(ps);
    drawArea()->setRenderer(mesh(), "Bounding Box");
    drawArea()->updateGL();
}

Q_EXPORT_PLUGIN(myresample)
