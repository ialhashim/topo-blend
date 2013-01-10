#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"
#include "StarlabDrawArea.h"

#include "NURBSCurve.h"
#include "NURBSRectangle.h"
#include "NurbsDraw.h"

#include "nurbstools.h"

#include "OBB_Volume.h"

class nurbs_plugin : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

public:
    QIcon icon(){ return QIcon(":/images/nurbs_icon.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();

    std::vector<NURBSCurve> curves;
    std::vector<NURBSRectangle> rects;

    NURBSTools * widget;

    Vector3VertexProperty points;

	OBB_Volume mesh_obb;

	void basicFit(NURBSCurve & curve, std::vector<Vec3d> pnts);
	void basicFitRecursive(NURBSCurve & curve, std::vector<Vec3d> pnts, int high, int low);

public slots:
    void doFitCurve();
    void doFitSurface();
};
