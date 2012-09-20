#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"

class geodistance : public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "Geodesic distance"; }
    QString description() { return "Compute geo-distance."; }

    void initParameters(RichParameterSet* pars);
    void applyFilter(RichParameterSet* pars);
};
