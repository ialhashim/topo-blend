#pragma once

#include "qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "topo_blend_widget.h"

class topoblend : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/topo-blend-icon.png"); }

public:
    topoblend();

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();

private:
    topo_blend_widget * widget;

public slots:

signals:

};
