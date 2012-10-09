#include "StructureCurve.h"
using namespace Structure;

Curve::Curve(NURBSCurve3d * newCurve, QString newID, QColor color)
{
    this->curve = newCurve;
    this->id = newID;

    this->vis_property["color"] = color;
}

QString Curve::type()
{
    return CURVE;
}

QBox3D Curve::bbox()
{
    QBox3D box;

    foreach(Vec3d cp, curve->getControlPoints())
        box.unite(cp);

    return box;
}

void Curve::draw()
{
    NurbsCurveDraw::draw( curve, vis_property["color"].value<QColor>() );
}

void Curve::draw2D()
{

}
