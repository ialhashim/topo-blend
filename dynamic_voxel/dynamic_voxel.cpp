#include "dynamic_voxel.h"
#include "StarlabDrawArea.h"

void dynamic_voxel::decorate(){
    vox.draw();
}

void dynamic_voxel::create(){

    bool showMesh = true;

    if(showMesh)
    {
        SurfaceMeshModel * m = new SurfaceMeshModel("voxel.obj", "voxel");
        vox.buildMesh( m );
        vox.MeanCurvatureFlow( m );
        document()->addModel(m);
    }

    drawArea()->updateGL();
}

Q_EXPORT_PLUGIN (dynamic_voxel)
