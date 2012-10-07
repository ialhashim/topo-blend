#include "dynamic_voxel.h"
#include "StarlabDrawArea.h"

void dynamic_voxel::decorate(){
    //vox.draw();
}

void dynamic_voxel::create(){

    SurfaceMeshModel * m = new SurfaceMeshModel("voxel.obj", "voxel");
    vox.buildMesh( m );
    document()->addModel(m);

    drawArea()->updateGL();
}

Q_EXPORT_PLUGIN (dynamic_voxel)
