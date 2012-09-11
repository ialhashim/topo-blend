#pragma once
#include "qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "resamplewidget.h"

class myresample : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/topo.png"); }

    /// Functions part of the EditPlugin system
	void createEdit();
    void destroyEdit(){}

    ResampleWidget * rw;

public slots:
    void doResample();

signals:

};
