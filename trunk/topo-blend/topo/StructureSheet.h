#pragma once

#include "StructureNode.h"
#include "NURBSRectangle.h"

namespace Structure{

struct Sheet : public Node
{
    // Constructors
    Sheet(NURBSRectangled * sheet, QString sheetID, QColor color = qRandomColor());

    // Underlaying structure
    NURBSRectangled * surface;

    // Properties
    QString type();
    QBox3D bbox();

    // Connections


    // Visualization
    void draw();
    void draw2D();
};

}
