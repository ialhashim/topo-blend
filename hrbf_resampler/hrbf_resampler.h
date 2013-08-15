#pragma once

#include "SurfaceMeshPlugins.h"
#include "RichParameterSet.h"

class hrbf_resampler: public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "HRBF Resampler"; }
    QString description() { return "Resample a mesh by reconstructing a surface defined by Hermite Radial Basis Function (HRBF)"; }

    void initParameters(RichParameterSet* pars);
    void applyFilter(RichParameterSet* pars);
};

