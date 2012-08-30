#pragma once
#include "SurfaceMeshPlugins.h"
#include "RichParameterSet.h"

class segment : public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "Segment"; }
    QString description() { return "Segment"; }

    void initParameters(RichParameterSet* pars);
    void applyFilter(RichParameterSet* pars);
};
