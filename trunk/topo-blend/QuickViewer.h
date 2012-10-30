#pragma once

#include <qglviewer/qglviewer.h>
#include "QuickMesh.h"

class QuickViewer : public QGLViewer{

public:
    QuickViewer(QuickMesh * using_mesh = NULL){
        this->m = using_mesh;
    }

    void draw(){
        if(m) m->draw();
    }

    QuickMesh * m;
};
