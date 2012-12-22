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
    void generateChairModels();
	void generateTwoSimpleModels();
    void loadModel();
	void clearGraphs();
    void doBlend();
	
	void experiment1();

	// Correspondence
	void visualizeFuzzyDistance(int sourceID);
	void findNodeCorrespondences();
	// End of Correspondence
signals:

};
