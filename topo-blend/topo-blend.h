#pragma once

#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "topo_blend_widget.h"

#include "StructureGraph.h"

class GraphCorresponder;

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

	void drawWithNames();
	void endSelection(const QPoint& p);
	void postSelection(const QPoint& p);

	bool keyPressEvent( QKeyEvent* event );

	QMap<QString, QVariant> params;
	QMap<QString, QVariant> viz_params;

public:
    topo_blend_widget * widget;

    QVector<Structure::Graph*> graphs;

	// DEBUG:
	QVector< Vector3 > debugPoints,debugPoints2,debugPoints3;
	QVector< QPair<Vector3,Vector3> > debugLines,debugLines2,debugLines3;
	Vector3VertexProperty points;
	
public:
	bool layout;

	// Corresponder
	GraphCorresponder *gcoor;
	GraphCorresponder* corresponder();
	Structure::Graph * getGraph(int id);

public slots:
	void setSceneBounds();

	// Generate
    void generateChairModels();
	void generateTwoSimpleModels();

	// Graphs
    void loadModel();
	void saveModel();
    void clearGraphs();
    void modifyModel();
    void quickAlign();

	void experiment1();
	void currentExperiment();

	// Main blend process
	void doBlend();

	// Experiments
	void experimentSlot();

	// Update
	void updateDrawArea();
	void testPoint2PointCorrespondences();

	// Show graphs
	void updateActiveGraph(Structure::Graph *);

	// Synthesis data
	void generateSynthesisData();
	void saveSynthesisData();
	void loadSynthesisData();
	void outputPointCloud();
	void genSynData();
	void reconstructXYZ();
	void combineMeshesToOne();

	void doRenderAll();
	void renderAll();
	void renderCurrent();
	void renderGraph(Structure::Graph *, QString filename, bool isOutPointCloud = false);
	void draftRender();

	void setStatusBarMessage(QString);
signals:
	void statusBarMessage(QString);
};
