#pragma once

#include "qglviewer/qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "geometry_morph_widget.h"

#include "NURBSCurve.h"
//#include "../topo-blend/topo/DynamicGraph.h"
#include "StructureGraph.h"
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

		void loadSourceCurve();
		void loadTargetCurve();

		void loadSourceGraph();
		void loadTargetGraph();

		void doMorph();

signals:

private:
	geometry_morph_widget * widget;
	
	SurfaceMeshModel * sourceModel;
	SurfaceMeshModel * targetModel;
	SurfaceMeshModel * blendModel;
	
	NURBSCurve sourceCurve;
	NURBSCurve targetCurve;
	//NURBSCurve blendCurve;

	Structure::Graph sourceGraph;
	Structure::Graph targetGraph;
};
