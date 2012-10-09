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


void Sheet::draw()
{
    NurbsSurfaceDraw::draw( surface, vis_property["color"].value<QColor>() );
}

void Sheet::draw2D()
{

}
