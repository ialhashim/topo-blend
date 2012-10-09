#pragma once

#include "StructureNode.h"
#include "NURBSCurve3.h"

namespace Structure{

struct Curve : public Node
{
    // Constructors
    Curve(NURBSCurve3d * newCurve, QString newID, QColor color = qRandomColor());

    // Underlaying structure
    NURBSCurve3d * curve;

    // Properties
    QString type();
    QBox3D bbox();

    // Connections


    // Visualization
    void draw();
    void draw2D();
};

}
