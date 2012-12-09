#pragma once

#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"

class voxel_resampler : public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "Voxel Resampler"; }
    QString description() { return "Resample a mesh by voxelizing it and then meshing"; }

    void initParameters(RichParameterSet* pars);
    void applyFilter(RichParameterSet* pars);
};
