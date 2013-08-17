#pragma once

#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

#include "topo_blend_widget.h"
#include "StructureGraph.h"
#include "StructureNode.h"

// Forward declare
class GraphCorresponder;
class TopoBlender;
class Scheduler;

class GraphsManager;
class CorrespondenceManager;
class SynthesisManager;

class topoblend : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/topo-blend-icon.png"); }

public:
    friend class topo_blend_widget;
    topo_blend_widget * widget;
    QVector<Structure::Graph*> graphs;
	Eigen::AlignedBox3d bigbox;

    // DEBUG:
    QVector< Vector3 > debugPoints,debugPoints2,debugPoints3;
    QVector< QPair<Vector3,Vector3> > debugLines,debugLines2,debugLines3;
    Vector3VertexProperty points;

public:

    // Corresponder
    GraphCorresponder *gcoor;
    Structure::Graph * getGraph(int id);

public:
    topoblend();

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();
	void drawBBox(Eigen::AlignedBox3d bbox);

    // Selection
    void drawWithNames();
    bool postSelection(const QPoint& p);

    // Mouse and Keyboard
	bool keyPressEvent( QKeyEvent* event );
	bool mouseReleaseEvent( QMouseEvent * event );

    Structure::Graph *orgSource;
    Structure::Graph *orgTarget;

	TopoBlender * blender;
	Scheduler * scheduler;

	QMap<QString, QVariant> params;
	QMap<QString, QVariant> viz_params;
	QMap<QString, QVariant> property;

    // Refactoring
    friend class GraphsManager;
    friend class CorrespondenceManager;
    friend class SynthesisManager;

    GraphsManager * g_manager;
    CorrespondenceManager * c_manager;
    SynthesisManager * s_manager;

public slots:

	// Main blend process
	void doBlend();

    // Update GUI
    void updateDrawArea();
    void setSceneBounds();
    void setStatusBarMessage(QString);

    // Show graphs
    void updateActiveGraph(Structure::Graph *);

signals:
	void statusBarMessage(QString);

};
