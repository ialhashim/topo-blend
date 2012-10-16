#pragma once

#include "qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "topo_blend_widget.h"

#include "topo/StructureGraph.h"

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

	bool keyPressEvent( QKeyEvent* event );

private:
    topo_blend_widget * widget;

    QVector<Structure::Graph> graphs;

public slots:
    void generateTwoModels();
    void loadModel();

signals:

};
