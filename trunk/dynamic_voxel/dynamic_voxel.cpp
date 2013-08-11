#include "dynamic_voxel.h"
#include "StarlabDrawArea.h"

void dynamic_voxel::decorate()
{
    if(showVoxels)
        vox.draw();
}

void dynamic_voxel::create(){

    showMesh = false;
    showVoxels = false;

    drawArea()->updateGL();
}

bool dynamic_voxel::keyPressEvent( QKeyEvent* event )
{
    bool used = false;

    if(event->key() == Qt::Key_V) {
        showVoxels = !showVoxels;

		Vector3d to(2,2,0);
		double radius = 0.5;

		vox.begin();

		QVector<Vector3d> points;
		points.push_back(Vector3d(-1,-1,0));
		points.push_back(Vector3d(0,-1,0));
		points.push_back(Vector3d(0,0,0));
		points.push_back(Vector3d(1,0,0));
		points.push_back(Vector3d(1,1,0));
		points.push_back(Vector3d(2,1,0));
		points.push_back(Vector3d(2,2,0));
		vox.addPolyLine(points, radius);

		vox.end();

        used = true;
    }

    if(event->key() == Qt::Key_M) {

        used = true;
    }

    drawArea()->updateGL();

    return used;
}

Q_EXPORT_PLUGIN (dynamic_voxel)
