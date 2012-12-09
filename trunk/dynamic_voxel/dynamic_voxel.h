#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"

#include "DynamicVoxel.h"

class dynamic_voxel : public SurfaceMeshModePlugin{
	Q_OBJECT
    Q_INTERFACES(ModePlugin)

public:
    QIcon icon(){ return QIcon(":/images/voxel_icon.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();

    DynamicVoxelLib::DynamicVoxel vox;

    bool showVoxels, showMesh;

public:
    virtual bool keyPressEvent(QKeyEvent* event);
};
