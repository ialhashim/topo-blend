#include "dynamic_voxel.h"
#include "StarlabDrawArea.h"

void dynamic_voxel::decorate()
{
    if(showVoxels)
        vox.draw();
}

void dynamic_voxel::create(){

    showMesh = true;
    showVoxels = false;

    if(showMesh)
    {
        SurfaceMeshModel * m = new SurfaceMeshModel("voxel_example.obj", "voxel");
        vox.buildMesh( m );
        vox.MeanCurvatureFlow( m );
        document()->addModel(m);
    }

    drawArea()->updateGL();
}

bool dynamic_voxel::keyPressEvent( QKeyEvent* event )
{
    bool used = false;

    if(event->key() == Qt::Key_V) {
        showVoxels = !showVoxels;
        used = true;
    }

    if(event->key() == Qt::Key_M) {
        showMesh = !showMesh;
        used = true;
    }

    drawArea()->updateGL();

    return used;
}

Q_EXPORT_PLUGIN (dynamic_voxel)
