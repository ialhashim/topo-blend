#pragma once

#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "topo_blend_widget.h"

#include "StructureGraph.h"

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

    QVector<Structure::Graph*> graphs;

	// DEBUG:
	QVector< Vector3 > debugPoints,debugPoints2,debugPoints3;
	QVector< QPair<Vector3,Vector3> > debugLines,debugLines2,debugLines3;

public slots:
	void setSceneBounds();

	// Generate
    void generateChairModels();
	void generateTwoSimpleModels();

	// Graphs
    void loadModel();
	void saveModel();
	void modifyModel();
	void clearGraphs();

	void experiment1();

	// Main blend process
	void doBlend();

	// Correspondence
	void visualizeFuzzyDistance(int sourceID);
	void findNodeCorrespondences();
	// End of Correspondence
signals:

};
