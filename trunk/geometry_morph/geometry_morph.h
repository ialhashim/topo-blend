#pragma once

#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "geometry_morph_widget.h"

#include "NURBSCurve.h"
//#include "../topo-blend/topo/DynamicGraph.h"
#include "StructureGraph.h"
#include "StructureNode.h"
//#include "CurveskelHelper.h"


class geometry_morph : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

public:
	geometry_morph();

	/// Functions part of the EditPlugin system
	void create();
	void destroy(){}

	void decorate();

	bool keyPressEvent( QKeyEvent* event );

	public slots:
		void setSceneBounds();
		
		void loadSourceModel();
		void loadTargetModel();

		void loadSourceGraph();
		void loadTargetGraph();

		void doMorph();

signals:

private:
	geometry_morph_widget * widget;

public:
	QVector<SurfaceMeshModel*> sourceModels;
	QVector<SurfaceMeshModel*> targetModels;

	QVector<Structure::Graph> sourceGraphs;
	QVector<Structure::Graph> targetGraphs;

	std::vector<SurfaceMeshModel*> resampledSourceModels;
	std::vector<SurfaceMeshModel*> resampledTargetModels;

	QVector<SurfaceMeshModel*> blendedModels;
};
