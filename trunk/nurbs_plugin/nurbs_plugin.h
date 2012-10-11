#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"
#include "StarlabDrawArea.h"

#include "NURBSCurve.h"
#include "NURBSRectangle.h"
#include "NurbsDraw.h"

class nurbs_plugin : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

public:
    QIcon icon(){ return QIcon(":/images/nurbs_icon.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();

    std::vector<NURBSCurve> nc;
    std::vector<NURBSRectangle> rects;
};
