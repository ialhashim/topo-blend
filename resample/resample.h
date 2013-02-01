#pragma once
#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "resamplewidget.h"

class myresample : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/resample.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

	int faceLimit;

public slots:
    void doResample();

    void doParameterize();

signals:

};
