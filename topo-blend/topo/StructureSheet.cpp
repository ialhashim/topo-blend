#include "StructureSheet.h"
using namespace Structure;

Sheet::Sheet(NURBSRectangled * sheet, QString sheetID, QColor color)
{
    this->surface = sheet;
    this->id = sheetID;

    this->vis_property["color"] = color;
}

QString Sheet::type()
{
    return SHEET;
}

QBox3D Sheet::bbox()
{
    QBox3D box;

    foreach(std::vector<Vec3d> cps, surface->mCtrlPoint)
        foreach(Vec3d cp, cps)
            box.unite(cp);

    return box;
}

void Sheet::draw()
{
    NurbsSurfaceDraw::draw( surface, vis_property["color"].value<QColor>() );
}

void Sheet::draw2D()
{

}
