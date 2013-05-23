#include <QElapsedTimer>
#include <QFileDialog>
#include <QDialog>
#include <QStack>
#include <QQueue>
#include <QtConcurrentRun>

#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"
#include "graph_modify_dialog.h"
#include "QuickAlignment.h"

using namespace NURBS;

// Graph manipulations
#include "DynamicGraph.h"
#include "ExportDynamicGraph.h"
#include "GraphDistance.h"
#include "TopoBlender.h"
#include "Scheduler.h"
#include "Task.h"
Q_DECLARE_METATYPE( Task* )

// Graph Correspondence
#include "GraphCorresponder.h"

// Synthesis
#include "Synthesizer.h"
#include "normal_extrapolation.h"
#include "SimilarSampling.h"

// Reconstruction
#include "poissonrecon.h"

VectorSoup vs1,vs2;

double boundX = -DBL_MAX;

Structure::Graph *orgSource = NULL, *orgTarget = NULL;

topoblend::topoblend()
{
	widget = NULL;
	gcoor = NULL;
	layout = true;
	blender = NULL;
	scheduler = NULL;
}

void topoblend::create()
{
	if(!widget)
	{
		ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
		widget = new topo_blend_widget(this);
		dockwidget->setWidget(widget);
		dockwidget->setWindowTitle(widget->windowTitle());
		mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);

		points = mesh()->vertex_property<Vector3>("v:point");

		// Events
		this->connect(this,SIGNAL(statusBarMessage(QString)),SLOT(setStatusBarMessage(QString)));
	}

	drawArea()->setSelectRegionHeight(20);
	drawArea()->setSelectRegionWidth(20);
}

void topoblend::decorate()
{
	// 2D view
	//glDisable(GL_LIGHTING);
	//for(int g = 0; g < (int) graphs.size(); g++)
	//{
	//	if(graphs[g]->edges.size() < 2) continue;

	//	drawArea()->startScreenCoordinatesSystem();
	//	graphs[g]->draw2D(150,150);
	//	drawArea()->stopScreenCoordinatesSystem();
	//}

	// DEBUG distance:
	for(int g = 0; g < (int) graphs.size(); g++){
		if(graphs[g]->misc.contains("distance")){
			GraphDistance * gd = (GraphDistance *)graphs[g]->misc["distance"];
			gd->draw();
		}
	}

	// DEBUG:
	glDisable(GL_LIGHTING);

	//glClear( GL_DEPTH_BUFFER_BIT );

	// Points 1
	glPushMatrix();
	glTranslatef(0, -drawArea()->sceneRadius(), 0);
	glPointSize(4); glColor3d(1,0,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints) glVector3(v); glEnd();
	glPointSize(8); glColor3d(1,1,1); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints) glVector3(v); glEnd();

	vs1.draw();

	glPopMatrix();

	// Points 2
	glPushMatrix();
	glTranslatef(0.0, drawArea()->sceneRadius(), 0);
	glColor3d(0,1,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints2) glVector3(v); glEnd();
	
	vs2.draw();

	glPopMatrix();

	glColor3d(0,0,1); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints3) glVector3(v); glEnd();

	// Lines
	typedef QPair<Vector3,Vector3> MyLine;
	glColor3d(1,0,0); glBegin(GL_LINES); foreach(MyLine l, debugLines) {glVector3(l.first);  glVector3(l.second);} glEnd();
	glColor3d(0,1,0); glBegin(GL_LINES); foreach(MyLine l, debugLines2) {glVector3(l.first); glVector3(l.second);} glEnd();
	glColor3d(0,0,1); glBegin(GL_LINES); foreach(MyLine l, debugLines3) {glVector3(l.first); glVector3(l.second);} glEnd();

	glEnable(GL_LIGHTING);

	if(blender) blender->drawDebug();

	// 3D visualization
	glEnable(GL_LIGHTING);

	double deltaX = boundX;
	double posX = -(deltaX / 2) * (graphs.size() / 2);


	for(int g = 0; g < (int) graphs.size(); g++)
	{
		// Apply visualization options
		graphs[g]->property["showEdges"] = viz_params["showEdges"];
		graphs[g]->property["showMeshes"] = viz_params["showMeshes"];
		graphs[g]->property["showTasks"] = viz_params["showTasks"];
		graphs[g]->property["showCtrlPts"] = viz_params["showCtrlPts"];
		graphs[g]->property["isSplatsHQ"] = viz_params["isSplatsHQ"];
		graphs[g]->property["splatSize"] = viz_params["splatSize"];
		graphs[g]->property["showNodes"] = viz_params["showNodes"];
		graphs[g]->property["showNodes"] = viz_params["showNodes"];
		graphs[g]->property["showNames"] = viz_params["showNames"];
		graphs[g]->property["showCurveFrames"] = viz_params["showCurveFrames"];

		// Place and draw graph
		glPushMatrix();
		glTranslatef(posX, 0, 0);
        graphs[g]->draw( drawArea() );
		glPopMatrix();

		posX += deltaX;
	}

	// Textual information
	glColor4d(1,1,1,0.25);
	drawArea()->renderText(40,40, "[TopoBlend]");
	if(property["correspondenceMode"].toBool()) 
	{
		glColor4d(1,1,1,1);
		drawArea()->renderText(40,80, "Correspondence Mode");
	}
}

void topoblend::drawWithNames()
{
	float deltaX = layout ? drawArea()->sceneRadius() : 0;
	float posX = - deltaX * (graphs.size() - 1) / 2;

	if(property["correspondenceMode"].toBool())
	{
		int offset = 0;

		for(int gID = 0; gID < (int) graphs.size(); gID++)
		{
			Structure::Graph *g = graphs[gID];

			glPushMatrix();
			glTranslatef(posX, 0, 0);

			g->drawNodeMeshNames( offset );

			glPopMatrix();

			posX += deltaX;
		}
		
		return;
	}

	// Select control points
	for(int gID = 0; gID < (int) graphs.size(); gID++)
	{
		Structure::Graph *g = graphs[gID];
		int nodeID_base = gID * NODE_ID_RANGE;

		glPushMatrix();
		glTranslatef(posX, 0, 0);

		for (int nID = 0; nID < (int)g->nodes.size(); nID++)
		{
			g->nodes[nID]->drawWithNames(nodeID_base + nID, POINT_ID_RANGE);
		}

		glPopMatrix();

		posX += deltaX;
	}
}

void topoblend::endSelection( const QPoint& p )
{
	drawArea()->defaultEndSelection(p);
}

void topoblend::postSelection( const QPoint& point )
{
    Q_UNUSED(point);
	int selectedID = drawArea()->selectedName();

	if(property["correspondenceMode"].toBool())
	{
		// mesh visualization - set to solid gray
		foreach(Graph * g, graphs){
			foreach(Node * n, g->nodes){
				int nodeID = n->property["meshSelectID"].toInt();

				if(n->property["is_corresponded"].toBool())
					continue;

				if( selectedID == nodeID )
				{
					if( n->property["nodeSelected"].toBool() )
					{
						n->property["nodeSelected"] = false;
						n->vis_property["meshColor"].setValue( QColor(180,180,180) );
					}
					else
					{
						n->property["nodeSelected"] = true;
						n->vis_property["meshColor"].setValue( QColor(50,70,255) );
					}
				}
			}
		}

		return;
	}

	if (selectedID == -1) return;

	int gID, nID, pID;
	getIndicesFromSelectedName(selectedID, gID, nID, pID);

	graphs[gID]->nodes[nID]->addSelectionWithColor(pID, Qt::green);

	qDebug() << "Selected ID is " << selectedID;
}

void topoblend::generateChairModels()
{
	QElapsedTimer assembleTimer; assembleTimer.start(); 

	Structure::Graph chair1, chair2, chair3, chair4, chair5;
	Structure::Graph swivel_chair1, swivel_chair2;

    Vector3 dU = Vector3(1,0,0);
    Vector3 dV = Vector3(0,0,1);

    NURBSRectangled backSheet = NURBSRectangled::createSheet(2,1, Vector3(0,-0.5,2), dU, dV);

    NURBSRectangled seatSheet = NURBSRectangled::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,1,0));
    NURBSRectangled seatSheetInv = NURBSRectangled::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,-1,0));

    NURBSCurved backLeft = NURBSCurved::createCurve(Vector3(-0.9,0,0), Vector3(-0.9,-0.5,1.5));
    NURBSCurved backRight = NURBSCurved::createCurve(Vector3(0.9,0,0), Vector3(0.9,-0.5,1.5));
    NURBSCurved backLeft2 = NURBSCurved::createCurve(Vector3(-0.5,0,0), Vector3(-0.5,-0.5,1.5));
    NURBSCurved backRight2 = NURBSCurved::createCurve(Vector3(0.5,0,0), Vector3(0.5,-0.5,1.5));
    NURBSCurved frontLegLeft = NURBSCurved::createCurve(Vector3(-1,1.75,0), Vector3(-1,1.9,-2));
    NURBSCurved frontLegRight = NURBSCurved::createCurve(Vector3(1,1.75,0), Vector3(1,1.9,-2));
    NURBSCurved backLegLeft = NURBSCurved::createCurve(Vector3(-1,0.25,0), Vector3(-1,0,-2));
    NURBSCurved backLegRight = NURBSCurved::createCurve(Vector3(1,0.25,0), Vector3(1,0,-2));
    NURBSCurved backTop = NURBSCurved::createCurve(Vector3(-0.9,-0.3345,1), Vector3(0.9,-0.3345,1));
    NURBSCurved backBottom = NURBSCurved::createCurve(Vector3(-0.9,-0.171,0.5), Vector3(0.9,-0.171,0.5));
    NURBSCurved legBarLeft = NURBSCurved::createCurve(Vector3(-1,0.126,-1), Vector3(-1,1.825,-1));
    NURBSCurved legBarMiddle = NURBSCurved::createCurve(Vector3(-1,1.825,-1), Vector3(1,1.825,-1));
    NURBSCurved legBarRight = NURBSCurved::createCurve(Vector3(1,0.126,-1), Vector3(1,1.825,-1));
    NURBSCurved extraBar = NURBSCurved::createCurve(Vector3(1,0.079,-1.36), Vector3(1,-0.2,-1.6));

    NURBSCurved swivel1MiddleBar = NURBSCurved::createCurve(Vector3(0,1,0), Vector3(0,1,-2));
    NURBSCurved swivel1Branch1 = NURBSCurved::createCurve(Vector3(0,1,-2), Vector3(1,1.9,-2));
    NURBSCurved swivel1Branch2 = NURBSCurved::createCurve(Vector3(0,1,-2), Vector3(-1,1.9,-2));
    NURBSCurved swivel1Branch3 = NURBSCurved::createCurve(Vector3(0,1,-2), Vector3(1,0,-2));
    NURBSCurved swivel1Branch4 = NURBSCurved::createCurve(Vector3(0,1,-2), Vector3(-1,0,-2));

    NURBSCurved swivel2MiddleBar = NURBSCurved::createCurve(Vector3(0,1,0), Vector3(0,1,-0.5));
    NURBSCurved swivel2Branch1 = NURBSCurved::createCurve(Vector3(0,1,-0.5), Vector3(1,1.9,-2));
    NURBSCurved swivel2Branch2 = NURBSCurved::createCurve(Vector3(0,1,-0.5), Vector3(-1,1.9,-2));
    NURBSCurved swivel2Branch3 = NURBSCurved::createCurve(Vector3(0,1,-0.5), Vector3(1,0,-2));
    NURBSCurved swivel2Branch4 = NURBSCurved::createCurve(Vector3(0,1,-0.5), Vector3(-1,0,-2));

if (0)
{
	// Chair 1
	chair1.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair1.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair1.addNode( new Structure::Curve(backRight, "BackRight") );
	chair1.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair1.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair1.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair1.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair1.addNode( new Structure::Curve(backLegRight, "BackLegRight") );

	// Add edges chair 1
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegRight") );

	// Chair 2
	chair2.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair2.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair2.addNode( new Structure::Curve(backLeft2, "BackLeft2") );
	chair2.addNode( new Structure::Curve(backRight2, "BackRight2") );
	chair2.addNode( new Structure::Curve(backRight, "BackRight") );
	chair2.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair2.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair2.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair2.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair2.addNode( new Structure::Curve(backLegRight, "BackLegRight") );

	// Add edges chair 2
	chair2.addEdge( chair2.getNode("BackSheet"), chair2.getNode("BackLeft") );
	chair2.addEdge( chair2.getNode("BackSheet"), chair2.getNode("BackRight") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackLeft") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackRight") );
	chair2.addEdge( chair2.getNode("BackSheet"), chair2.getNode("BackLeft2") );
	chair2.addEdge( chair2.getNode("BackSheet"), chair2.getNode("BackRight2") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackLeft2") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackRight2") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("FrontLegLeft") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("FrontLegRight") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackLegLeft") );
	chair2.addEdge( chair2.getNode("SeatSheet"), chair2.getNode("BackLegRight") );

	// Chair 3
	chair3.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair3.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair3.addNode( new Structure::Curve(backRight, "BackRight") );
	chair3.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair3.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair3.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair3.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair3.addNode( new Structure::Curve(backLegRight, "BackLegRight") );
	chair3.addNode( new Structure::Curve(backTop, "BackTop") );
	chair3.addNode( new Structure::Curve(backBottom, "BackBottom") );

	// Add edges chair 3
	chair3.addEdge( chair3.getNode("BackSheet"), chair3.getNode("BackLeft") );
	chair3.addEdge( chair3.getNode("BackSheet"), chair3.getNode("BackRight") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("BackLeft") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("BackRight") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("FrontLegLeft") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("FrontLegRight") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("BackLegLeft") );
	chair3.addEdge( chair3.getNode("SeatSheet"), chair3.getNode("BackLegRight") );
	chair3.addEdge( chair3.getNode("BackTop"), chair3.getNode("BackLeft") );
	chair3.addEdge( chair3.getNode("BackTop"), chair3.getNode("BackRight") );
	chair3.addEdge( chair3.getNode("BackBottom"), chair3.getNode("BackLeft") );
	chair3.addEdge( chair3.getNode("BackBottom"), chair3.getNode("BackRight") );

	// Chair 4
	chair4.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair4.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair4.addNode( new Structure::Curve(backRight, "BackRight") );
	chair4.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair4.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair4.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair4.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair4.addNode( new Structure::Curve(backLegRight, "BackLegRight") );
	chair4.addNode( new Structure::Curve(backTop, "BackTop") );
	chair4.addNode( new Structure::Curve(backBottom, "BackBottom") );
	chair4.addNode( new Structure::Curve(legBarLeft, "LegBarLeft") );
	chair4.addNode( new Structure::Curve(legBarMiddle, "LegBarMiddle") );
	chair4.addNode( new Structure::Curve(legBarRight, "LegBarRight") );

	// Add edges chair 4
	chair4.addEdge( chair4.getNode("BackSheet"), chair4.getNode("BackLeft") );
	chair4.addEdge( chair4.getNode("BackSheet"), chair4.getNode("BackRight") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("BackLeft") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("BackRight") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("FrontLegLeft") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("FrontLegRight") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("BackLegLeft") );
	chair4.addEdge( chair4.getNode("SeatSheet"), chair4.getNode("BackLegRight") );
	chair4.addEdge( chair4.getNode("BackTop"), chair4.getNode("BackLeft") );
	chair4.addEdge( chair4.getNode("BackTop"), chair4.getNode("BackRight") );
	chair4.addEdge( chair4.getNode("BackBottom"), chair4.getNode("BackLeft") );
	chair4.addEdge( chair4.getNode("BackBottom"), chair4.getNode("BackRight") );
	chair4.addEdge( chair4.getNode("BackLegLeft"), chair4.getNode("LegBarLeft") );
	chair4.addEdge( chair4.getNode("FrontLegLeft"), chair4.getNode("LegBarLeft") );
	chair4.addEdge( chair4.getNode("FrontLegLeft"), chair4.getNode("LegBarMiddle") );
	chair4.addEdge( chair4.getNode("FrontLegRight"), chair4.getNode("LegBarMiddle") );
	chair4.addEdge( chair4.getNode("FrontLegRight"), chair4.getNode("LegBarRight") );
	chair4.addEdge( chair4.getNode("BackLegRight"), chair4.getNode("LegBarRight") );

	// Chair 5
	chair5.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair5.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair5.addNode( new Structure::Curve(backRight, "BackRight") );
	chair5.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair5.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair5.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair5.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair5.addNode( new Structure::Curve(backLegRight, "BackLegRight") );
	chair5.addNode( new Structure::Curve(backTop, "BackTop") );
	chair5.addNode( new Structure::Curve(backBottom, "BackBottom") );
	chair5.addNode( new Structure::Curve(legBarLeft, "LegBarLeft") );
	chair5.addNode( new Structure::Curve(legBarMiddle, "LegBarMiddle") );
	chair5.addNode( new Structure::Curve(legBarRight, "LegBarRight") );
	chair5.addNode( new Structure::Curve(extraBar, "ExtraBar") );

	// Add edges chair 5
	chair5.addEdge( chair5.getNode("BackSheet"), chair5.getNode("BackLeft") );
	chair5.addEdge( chair5.getNode("BackSheet"), chair5.getNode("BackRight") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("BackLeft") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("BackRight") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("FrontLegLeft") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("FrontLegRight") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("BackLegLeft") );
	chair5.addEdge( chair5.getNode("SeatSheet"), chair5.getNode("BackLegRight") );
	chair5.addEdge( chair5.getNode("BackTop"), chair5.getNode("BackLeft") );
	chair5.addEdge( chair5.getNode("BackTop"), chair5.getNode("BackRight") );
	chair5.addEdge( chair5.getNode("BackBottom"), chair5.getNode("BackLeft") );
	chair5.addEdge( chair5.getNode("BackBottom"), chair5.getNode("BackRight") );
	chair5.addEdge( chair5.getNode("BackLegLeft"), chair5.getNode("LegBarLeft") );
	chair5.addEdge( chair5.getNode("FrontLegLeft"), chair5.getNode("LegBarLeft") );
	chair5.addEdge( chair5.getNode("FrontLegLeft"), chair5.getNode("LegBarMiddle") );
	chair5.addEdge( chair5.getNode("FrontLegRight"), chair5.getNode("LegBarMiddle") );
	chair5.addEdge( chair5.getNode("FrontLegRight"), chair5.getNode("LegBarRight") );
	chair5.addEdge( chair5.getNode("BackLegRight"), chair5.getNode("LegBarRight") );
	chair5.addEdge( chair5.getNode("BackLegRight"), chair5.getNode("ExtraBar") );

	chair1.saveToFile("chair1.xml");
	chair2.saveToFile("chair2.xml");
	chair3.saveToFile("chair3.xml");
	chair4.saveToFile("chair4.xml");
	chair5.saveToFile("chair5.xml");

	// Swivel Chair 1
	swivel_chair1.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	swivel_chair1.addNode( new Structure::Curve(backLeft, "BackLeft") );
	swivel_chair1.addNode( new Structure::Curve(backRight, "BackRight") );
	swivel_chair1.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	swivel_chair1.addNode( new Structure::Curve(swivel1MiddleBar, "MiddleBar") );
	swivel_chair1.addNode( new Structure::Curve(swivel1Branch1, "Branch1") );
	swivel_chair1.addNode( new Structure::Curve(swivel1Branch2, "Branch2") );
	swivel_chair1.addNode( new Structure::Curve(swivel1Branch3, "Branch3") );
	swivel_chair1.addNode( new Structure::Curve(swivel1Branch4, "Branch4") );

	// Add edges
	swivel_chair1.addEdge( swivel_chair1.getNode("BackSheet"), swivel_chair1.getNode("BackLeft") );
	swivel_chair1.addEdge( swivel_chair1.getNode("BackSheet"), swivel_chair1.getNode("BackRight") );
	swivel_chair1.addEdge( swivel_chair1.getNode("SeatSheet"), swivel_chair1.getNode("BackLeft") );
	swivel_chair1.addEdge( swivel_chair1.getNode("SeatSheet"), swivel_chair1.getNode("BackRight") );
	swivel_chair1.addEdge( swivel_chair1.getNode("SeatSheet"), swivel_chair1.getNode("MiddleBar") );
	swivel_chair1.addEdge( swivel_chair1.getNode("MiddleBar"), swivel_chair1.getNode("Branch1") );
	swivel_chair1.addEdge( swivel_chair1.getNode("MiddleBar"), swivel_chair1.getNode("Branch2") );
	swivel_chair1.addEdge( swivel_chair1.getNode("MiddleBar"), swivel_chair1.getNode("Branch3") );
	swivel_chair1.addEdge( swivel_chair1.getNode("MiddleBar"), swivel_chair1.getNode("Branch4") );


	// Swivel Chair 2
	swivel_chair2.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	swivel_chair2.addNode( new Structure::Curve(backLeft, "BackLeft") );
	swivel_chair2.addNode( new Structure::Curve(backRight, "BackRight") );
	swivel_chair2.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	swivel_chair2.addNode( new Structure::Curve(swivel2MiddleBar, "MiddleBar") );
	swivel_chair2.addNode( new Structure::Curve(swivel2Branch1, "Branch1") );
	swivel_chair2.addNode( new Structure::Curve(swivel2Branch2, "Branch2") );
	swivel_chair2.addNode( new Structure::Curve(swivel2Branch3, "Branch3") );
	swivel_chair2.addNode( new Structure::Curve(swivel2Branch4, "Branch4") );

	// Add edges
	swivel_chair2.addEdge( swivel_chair2.getNode("BackSheet"), swivel_chair2.getNode("BackLeft") );
	swivel_chair2.addEdge( swivel_chair2.getNode("BackSheet"), swivel_chair2.getNode("BackRight") );
	swivel_chair2.addEdge( swivel_chair2.getNode("SeatSheet"), swivel_chair2.getNode("BackLeft") );
	swivel_chair2.addEdge( swivel_chair2.getNode("SeatSheet"), swivel_chair2.getNode("BackRight") );
	swivel_chair2.addEdge( swivel_chair2.getNode("SeatSheet"), swivel_chair2.getNode("MiddleBar") );
	swivel_chair2.addEdge( swivel_chair2.getNode("MiddleBar"), swivel_chair2.getNode("Branch1") );
	swivel_chair2.addEdge( swivel_chair2.getNode("MiddleBar"), swivel_chair2.getNode("Branch2") );
	swivel_chair2.addEdge( swivel_chair2.getNode("MiddleBar"), swivel_chair2.getNode("Branch3") );
	swivel_chair2.addEdge( swivel_chair2.getNode("MiddleBar"), swivel_chair2.getNode("Branch4") );

	// Save to file
	swivel_chair1.saveToFile("swivelChair1.xml");
	swivel_chair2.saveToFile("swivelChair2.xml");
}
	//graphs.push_back(chair1);
	//graphs.push_back(chair2);

	qDebug() << "Chairs structure graphs construction " << assembleTimer.elapsed() << " ms.";

	setSceneBounds();
}

void topoblend::generateTwoSimpleModels()
{
	QElapsedTimer assembleTimer; assembleTimer.start(); 

	Structure::Graph model1, model2;

    Vector3 dU = Vector3(1,0,0);
    Vector3 dV = Vector3(0,0,1);

	// Geometry
    NURBSRectangled centerSheet = NURBSRectangled::createSheet(1,1, Vector3(0,0,0), dU, dV);
    NURBSRectangled leftSheet = NURBSRectangled::createSheet(1,1, Vector3(-3,0,2), dU, dV);
    NURBSRectangled rightSheet = NURBSRectangled::createSheet(1,1, Vector3(3,0,2), dU, dV);
    NURBSRectangled bottomSheet = NURBSRectangled::createSheet(1,1, Vector3(0,0,-2.5), dU, dV);

    NURBSCurved leftCurve = NURBSCurved::createCurve(Vector3(-0.5,0,0.5), Vector3(-2.5, 0, 1.5));
    NURBSCurved rightCurve = NURBSCurved::createCurve(Vector3(0.5,0,0.5), Vector3(2.5, 0, 1.5));
    NURBSCurved bottomCurve = NURBSCurved::createCurve(Vector3(0,0,-0.5), Vector3(0, 0, -2.0));
    NURBSCurved bottomLeftCurve = NURBSCurved::createCurve(Vector3(-2.5, 0, 1.5), Vector3(-0.5,0,-2));
    NURBSCurved bottomRightCurve = NURBSCurved::createCurve(Vector3(2.5, 0, 1.5), Vector3(0.5,0,-2));

	// Add nodes model 1
	model1.addNode( new Structure::Sheet(centerSheet, "Center") );
	model1.addNode( new Structure::Sheet(leftSheet, "Left") );
	model1.addNode( new Structure::Sheet(rightSheet, "Right") );
	model1.addNode( new Structure::Sheet(bottomSheet, "Bottom") );

	model1.addNode( new Structure::Curve(leftCurve, "LeftCurve") );
	model1.addNode( new Structure::Curve(rightCurve, "RightCurve") );
	model1.addNode( new Structure::Curve(bottomCurve, "BottomCurve") );

	// Add edges model 1
	model1.addEdge("LeftCurve", "Center");
	model1.addEdge("RightCurve", "Center");
	model1.addEdge("RightCurve", "Right");
	model1.addEdge("LeftCurve", "Left");
	model1.addEdge("BottomCurve", "Center");
	model1.addEdge("BottomCurve", "Bottom");

	// Add nodes model 2
	model2.addNode( new Structure::Sheet(centerSheet, "Center") );
	model2.addNode( new Structure::Sheet(leftSheet, "Left") );
	model2.addNode( new Structure::Sheet(rightSheet, "Right") );
	model2.addNode( new Structure::Sheet(bottomSheet, "Bottom") );

	model2.addNode( new Structure::Curve(leftCurve, "LeftCurve") );
	model2.addNode( new Structure::Curve(rightCurve, "RightCurve") );
	model2.addNode( new Structure::Curve(bottomCurve, "BottomCurve") );

	model2.addNode( new Structure::Curve(bottomLeftCurve, "LeftBottomCurve") );
	model2.addNode( new Structure::Curve(bottomRightCurve, "RightBottomCurve") );

	// Add edges model 1
	model2.addEdge("LeftCurve", "Center");
	model2.addEdge("RightCurve", "Center");
	model2.addEdge("RightCurve", "Right");
	model2.addEdge("LeftCurve", "Left");
	model2.addEdge("BottomCurve", "Center");
	model2.addEdge("BottomCurve", "Bottom");

	model2.addEdge("LeftBottomCurve", "Bottom");
	model2.addEdge("RightBottomCurve", "Bottom");
	model2.addEdge("LeftBottomCurve", "Left");
	model2.addEdge("RightBottomCurve", "Right");

	// Save to file
	model1.saveToFile("simple_model1.xml");
	model2.saveToFile("simple_model2.xml");

	//graphs.push_back(model1);
	//graphs.push_back(model2);

	qDebug() << "Simple models structure graphs construction " << assembleTimer.elapsed() << " ms.";

	setSceneBounds();
}

void topoblend::setSceneBounds()
{
	if(!graphs.size()) return;

	// Set scene bounds
	boundX = -DBL_MAX;

	QBox3D bigbox = graphs.front()->bbox();
	for(int i = 0; i < (int)graphs.size(); i++)
	{
		bigbox.unite( graphs[i]->bbox() );
		boundX = qMax(boundX, graphs[i]->bbox().size().x());
	}

	bigbox.transform(QMatrix4x4() * 3);

	Vector3 a = bigbox.minimum();
	Vector3 b = bigbox.maximum();

	qglviewer::Vec vecA(a.x(), a.y(), a.z());
	qglviewer::Vec vecB(b.x(), b.y(), b.z());

	drawArea()->setSceneCenter((vecA + vecB) * 0.5);
	drawArea()->setSceneBoundingBox(vecA, vecB);
	drawArea()->showEntireScene();
	drawArea()->updateGL();
}

void topoblend::loadModel()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));
	if(fileNames.isEmpty()) return;

	// Keep folder active
	QFileInfo fileInfo(fileNames.front());
	mainWindow()->settings()->set( "lastUsedDirectory", fileInfo.absolutePath() );

	foreach(QString file, fileNames)
	{	
		graphs.push_back( new Structure::Graph ( file ) );
	}

	mainWindow()->setStatusBarMessage( "Loaded: \n" + fileNames.join("\n") );

	setSceneBounds();
}

void topoblend::saveModel()
{
	if (graphs.size() < 1){
		qDebug() << "Please load a graph.";
		return;
	}

	foreach(Structure::Graph * g, graphs)
		g->setColorAll(Qt::lightGray);

	foreach(Structure::Graph * g, graphs)
	{
		// Highlight selected graph
		g->setColorAll(Qt::yellow);
		drawArea()->updateGL();

		QString filename = QFileDialog::getSaveFileName(0, tr("Save Model"), 
			g->property["name"].toString(), tr("Model Files (*.xml)"));

		// Un-highlight
		g->setColorAll(Qt::lightGray);

		if(filename.isEmpty()) 	continue;

		g->saveToFile(filename);
	}

}

void topoblend::modifyModel()
{
	if (graphs.size() < 1){
		qDebug() << "Please load a graph";
		return;
	}
	
	widget->setCheckOption("showEdges");

	viz_params["showNames"] = true;

    GraphModifyDialog modifyDialog(graphs.back());
	drawArea()->connect(&modifyDialog, SIGNAL(updateView()), SLOT(updateGL()));
    modifyDialog.exec();
}

void topoblend::quickAlign()
{
    if (graphs.size() < 1){
        qDebug() << "Please load a graph";
        return;
    }

    QuickAlignment alignmentDialog(graphs[0], graphs[1]);
    drawArea()->connect(&alignmentDialog, SIGNAL(updateView()), SLOT(updateGL()));
    alignmentDialog.exec();
}

bool topoblend::mouseReleaseEvent( QMouseEvent * event )
{
	bool used = false;

	if( property["correspondenceMode"].toBool() ) 
	{ 
		if( event->button() == Qt::RightButton )
		{
			Graph * sourceGraph = graphs.front();
			Graph * targetGraph = graphs.back();

			QVector<QString> sParts, tParts;

			QColor curColor = qRandomColor3(0, 0.25);

			// Source
			foreach(Node * n, sourceGraph->nodes){
				if( n->property["nodeSelected"].toBool() ){
					sParts << n->id;
					n->property["nodeSelected"] = false;
					n->vis_property["meshColor"].setValue( curColor );
				}
			}

			// Target
			foreach(Node * n, targetGraph->nodes){
				if( n->property["nodeSelected"].toBool() ){
					tParts << n->id;
					n->property["nodeSelected"] = false;
					n->vis_property["meshColor"].setValue( curColor );
				}
			}
			
			// Correspondence specified
			if(sParts.size() > 0 && tParts.size() > 0)
			{
				gcoor->addLandmarks(sParts, tParts);

				// Assign colors
				foreach(QString nodeID, sParts)	
				{
					sourceGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );
					sourceGraph->getNode( nodeID )->property["is_corresponded"] = true;
				}
				foreach(QString nodeID, tParts)	
				{
					targetGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );
					targetGraph->getNode( nodeID )->property["is_corresponded"] = true;
				}
			}
		}
	}

	return used;
}

bool topoblend::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	QElapsedTimer timer; timer.start();

	if(event->key() == Qt::Key_C)
	{
		foreach(Graph * g, graphs){
			foreach(Node * n, g->nodes){
				n->vis_property["meshColor"].setValue( QColor(180,180,180) );
				n->property["is_corresponded"] = false;
			}
		}

		gcoor->clear();

		used = true;
	}

	if(event->key() == Qt::Key_Space)
	{
		// Enter / exit correspondence mode
		property["correspondenceMode"] = !property["correspondenceMode"].toBool();
		if(!property["correspondenceMode"].toBool()) 
		{ 
			drawArea()->setMouseBinding(Qt::LeftButton, QGLViewer::CAMERA, QGLViewer::ROTATE);
			drawArea()->setMouseBinding(Qt::SHIFT | Qt::LeftButton, QGLViewer::SELECT);

			// Reset mesh visualization
			foreach(Graph * g, graphs){
				foreach(Node * n, g->nodes){
					n->vis_property["meshSolid"] = false;
					n->vis_property["meshColor"].setValue( QColor(200,200,200,8) );
				}
			}

			drawArea()->updateGL(); 
			return used; 
		}

		drawArea()->setMouseBinding(Qt::LeftButton, QGLViewer::SELECT);
		drawArea()->setMouseBinding(Qt::SHIFT | Qt::LeftButton, QGLViewer::CAMERA, QGLViewer::ROTATE);

		// mesh visualization - set to solid gray
		foreach(Graph * g, graphs){
			foreach(Node * n, g->nodes){
				n->vis_property["meshSolid"] = true;
				n->vis_property["meshColor"].setValue( QColor(180,180,180) );
			}
		}

		used = true;
	}

	if(event->key() == Qt::Key_Space && (event->modifiers() & Qt::ShiftModifier))
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			graphs[g]->materialize(0);
		}

		qDebug() << "Materialized graphs (voxel only) " << timer.elapsed() << " ms";

		used = true;
	}

	if(event->key() == Qt::Key_E)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			graphs[g]->cached_mesh.clear();

			SurfaceMesh::Model * m = new SurfaceMesh::Model( QString("Voxel_%1.obj").arg(g), QString("Voxel_%1").arg(g) );
			graphs[g]->materialize(m);
			DynamicVoxel::MeanCurvatureFlow(m, 0.05);
			//DynamicVoxel::LaplacianSmoothing(m);

			document()->pushBusy();
			document()->addModel(m);
			document()->popBusy();
		}

		qDebug() << "Materialized graphs, generated smooth mesh " << timer.elapsed() << " ms";

		used = true;
	}

	if(event->key() == Qt::Key_W)
	{
		if(graphs.size() < 1) return true;

		Structure::Graph * g = graphs.back();

		GraphDistance * gd = new GraphDistance(g);

		std::vector<Vector3> starts;
		starts.push_back(Vector3(1,-0.5,2.5));
		starts.push_back(Vector3(-1,-0.5,2.5));
		gd->computeDistances(starts, g->bbox().size().length() * 0.01);

		g->misc["distance"] = gd;

		mainWindow()->setStatusBarMessage("Distance test");

		used = true;
	}

	if(event->key() == Qt::Key_M)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
			graphs[g]->printAdjacency();
		used = true;
	}

	if(event->key() == Qt::Key_N)
	{
		viz_params["showNames"] = !viz_params["showNames"].toBool();

		used = true;
	}

	if(event->key() == Qt::Key_R)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			qDebug() << "Root (by valence) = " << graphs[g]->rootByValence()->id;
			qDebug() << "Root (by size) = " << graphs[g]->rootBySize()->id;
		}
		used = true;
	}
	
	if(event->key() == Qt::Key_T)
	{
		qglviewer::Vec q = drawArea()->camera()->revolveAroundPoint();
		Vec3d p(q[0],q[1],q[2]);

		double minDist = DBL_MAX;
		QString sample_details = "";

		double deltaX = boundX;
		double posX = -(deltaX / 2) * (graphs.size() / 2);

		for(int gi = 0; gi < (int) graphs.size(); gi++)
		{
			Structure::Graph * g = graphs[gi];
			g->property.remove("selectedSample");

			foreach(Structure::Node * node, g->nodes)
			{
				if(node->property.contains("cached_points"))
				{
					QVector<Vec3f> pnts = node->property["cached_points"].value< QVector<Vec3f> >();
					QVector<ParameterCoord> samples = node->property["samples"].value< QVector<ParameterCoord> >();
					QVector<float> offsets = node->property["offsets"].value< QVector<float> >();

					Vec3d delta = Vec3d(posX,0,0);
					Vec3d q = p - delta;

					for(int i = 0; i < (int)pnts.size(); i++)
					{
						double dist = (pnts[i] - q).norm();
						if(dist < minDist && dist < 0.01){
							ParameterCoord s = samples[i];
							sample_details = QString("[%5] u= %1  v= %2  theta= %3  psi= %4 offset = %6").arg(s.u
								).arg(s.v).arg(s.theta).arg(s.psi).arg(node->id).arg(offsets[i]);
							minDist = dist;
							g->property["selectedSample"].setValue(pnts[i]);
							g->property["selectedNode"].setValue(node->id);

							if(s.origNode) sample_details += QString(" [orig %1][").arg(s.origNode->id);
							
							QString graphName = g->property["name"].toString().section('\\', -1).section('.', 0, 0);
							sample_details = "["+graphName+"] " + sample_details;
						}
					}
				}
			}

			posX += deltaX;
		}

		setStatusBarMessage(sample_details);

		used = true;
	}

	if(event->key() == Qt::Key_J)
	{
		QString key = "ViewerStateFile";
		QString value = qApp->applicationDirPath() + "/viewer_state.xml";

		drawArea()->settings()->set( key, value );
		drawArea()->settings()->sync();

		drawArea()->setStateFileName( value );
		drawArea()->saveStateToFile();
		mainWindow()->setStatusBarMessage("Viewer state saved to file.");
		used = true;
	}

	if(event->key() == Qt::Key_K)
	{
		QString key = "ViewerStateFile";
		QString value = drawArea()->settings()->getString(key);

		drawArea()->setStateFileName( value );
		drawArea()->restoreStateFromFile();
		mainWindow()->setStatusBarMessage("Viewer state loaded.");
		used = true;
	}

	if(event->key() == Qt::Key_F)
	{  
		SurfaceMeshHelper h(mesh());
		Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

		Vertex vert = SurfaceMesh::Vertex(3);

		Vec3d center(0);
		foreach(Halfedge he, mesh()->onering_hedges(vert)){
			center += points[mesh()->to_vertex(he)];
		}
		center /= mesh()->valence(vert);

		points[vert] = AlphaBlend(0.1, points[vert], center);

		QVector<Vector3> samples = SimilarSampler::FaceSamples(mesh());

		PointSoup * ps = new PointSoup;
		drawArea()->deleteAllRenderObjects();
		foreach(Vector3 p, samples)	ps->addPoint(p);
		drawArea()->addRenderObject(ps);

		used = true;
	}

	if(event->key() == Qt::Key_G)
	{
		if(scheduler && blender)
		{
			QDir dir("");
			dir.setCurrent(QFileDialog::getExistingDirectory());

			QString sGraphName = gcoor->sgName();
			QString tGraphName = gcoor->tgName();

			// Export source and target
			DynamicGraphs::DynamicGraph sdg(blender->sg);
			DynamicGraphs::DynamicGraph tdg(blender->tg);
			toGraphviz(sdg, sGraphName, true, QString("V = %1, E = %2").arg(sdg.nodes.size()).arg(sdg.edges.size()), "original source");
			toGraphviz(tdg, tGraphName, true, QString("V = %1, E = %2").arg(tdg.nodes.size()).arg(tdg.edges.size()), "original target");

			// Export supers
			DynamicGraphs::DynamicGraph sdg_super(scheduler->activeGraph);
			DynamicGraphs::DynamicGraph tdg_super(scheduler->targetGraph);
			toGraphviz(sdg_super, sGraphName + "_super", true, QString("V = %1, E = %2").arg(sdg_super.nodes.size()).arg(sdg_super.edges.size()), "super source");
			toGraphviz(tdg_super, tGraphName + "_super", true, QString("V = %1, E = %2").arg(tdg_super.nodes.size()).arg(tdg_super.edges.size()), "super target");

			QImage img1(sGraphName + "_super"), img2(tGraphName + "_super");
			QImage bothImg(img1.width() + img2.width(), qMax(img1.height(), img2.height()), QImage::Format_RGB32);
			bothImg.fill(Qt::white);
			QPainter paint;
			paint.begin(&bothImg);
			paint.drawImage(0,0,img1);
			paint.drawImage(img1.width(),0,img2);
			paint.end();
			bothImg.save("All_Graphs.png");
		}
		used = true;
	}

	drawArea()->updateGL();

	return used;
}

void topoblend::experimentSlot()
{

}

void topoblend::doBlend()
{
	if ( graphs.size() < 2 && !orgSource )
	{
		qDebug() << "Please load at least two graphs.";
		return;
	}

	// Visualization
	foreach(Graph * g, graphs){
		foreach(Node * n, g->nodes){
			n->vis_property["meshSolid"] = false;
			n->vis_property["meshColor"].setValue( QColor(200,200,200,8) );
		}
	}

	// Assign newly loaded source and target
	if( graphs.size() == 2 && !orgSource && !orgTarget )
	{
		orgSource = new Graph(*graphs.front());
		orgTarget = new Graph(*graphs.back());
	}

	QElapsedTimer timer; timer.start();

	// Reload previous source and target if any
	if( graphs.size() < 2 )
	{
		// Use previously loaded 
		this->graphs.clear();
		this->graphs.push_back( new Graph(*orgSource) );
		this->graphs.push_back( new Graph(*orgTarget) );
	}

	if(scheduler) 
	{
		// Old signals
		scheduler->disconnect(this);
		this->disconnect(scheduler);
		
		scheduler->dock->close();

		scheduler = NULL;
	}

	Structure::Graph * source = graphs.front();
	Structure::Graph * target = graphs.back();

	if( !gcoor )
	{
		gcoor = makeCorresponder();
		gcoor->computeCorrespondences();
	}

	scheduler = new Scheduler();
    blender = new TopoBlender( gcoor, scheduler );

	// Update active graph
	this->connect(scheduler, SIGNAL(activeGraphChanged( Structure::Graph* )), SLOT(updateActiveGraph( Structure::Graph* )));

	// Render connections
	this->connect(scheduler, SIGNAL(renderAll()), SLOT(renderAll()), Qt::UniqueConnection);
	this->connect(scheduler, SIGNAL(renderCurrent()), SLOT(renderCurrent()), Qt::UniqueConnection);
	this->connect(scheduler, SIGNAL(draftRender()), SLOT(draftRender()), Qt::UniqueConnection);

	setStatusBarMessage( QString("Created TopoBlender and tasks in [ %1 ms ]").arg( timer.elapsed() ) );

	//this->graphs.clear();
	//this->graphs.push_back(scheduler->activeGraph);
	//this->graphs.push_back(scheduler->targetGraph);

	setSceneBounds();

	updateDrawArea();
}

void topoblend::experiment1()
{

}

void topoblend::clearGraphs()
{
	drawArea()->updateGL();

	if(scheduler) 
	{
		scheduler->cleanUp();

		// Old signals
		scheduler->disconnect(this);
		this->disconnect(scheduler);

		scheduler->dock->close();
	}
	scheduler = NULL;

	blender = NULL;

	delete gcoor;
	gcoor = NULL;

	orgSource = orgTarget = NULL;

	debugPoints.clear(); debugPoints2.clear();	
	
	qDeleteAll(graphs);
	graphs.clear();
}

void topoblend::currentExperiment()
{

}

void topoblend::updateDrawArea()
{
	drawArea()->updateGL();
}

GraphCorresponder* topoblend::makeCorresponder()
{
	if (graphs.size() < 2)
	{
		qDebug() << "Please load at least two graphs.";
		return NULL;
	}

	Structure::Graph *sg = graphs[0];
	Structure::Graph *tg = graphs[1];

	gcoor = new GraphCorresponder(sg, tg);

	return gcoor;
}

void topoblend::testPoint2PointCorrespondences()
{
	drawArea()->updateGL();
}

Structure::Graph * topoblend::getGraph(int id)
{
	if (id >= 0 && id < graphs.size())
		return graphs[id];
	else
		return NULL;
}
void topoblend::updateActiveGraph( Structure::Graph * newActiveGraph )
{
	// Debug:
	if( blender )
	{
		if( false )
		{
			if( !newActiveGraph->nodes.front()->property.contains("samples") )
			{
				viz_params["showMeshes"] = false;
				viz_params["showTasks"] = true;
			}
		}

		//newActiveGraph->normalize();
		//newActiveGraph->moveBottomCenterToOrigin();
	}	
	
	//setSceneBounds();

	graphs.clear();
	this->graphs.push_back(newActiveGraph);
	drawArea()->updateGL();
}

void topoblend::genSynData()
{
	drawArea()->camera()->setType(qglviewer::Camera::PERSPECTIVE);

	qApp->setOverrideCursor(Qt::WaitCursor);

	QElapsedTimer timer; timer.start();

	int numNodes = scheduler->activeGraph->nodes.size();
	int n = 0;

	// Force generate
	foreach(Structure::Node * node, scheduler->activeGraph->nodes)
	{
		if(node->property.contains("correspond"))
		{
			QString nodeID = node->id;
			QString tnodeID = node->property["correspond"].toString();
			Structure::Node * tgNode = scheduler->targetGraph->getNode(tnodeID);
			Synthesizer::clearSynthData(node); 
			Synthesizer::clearSynthData(tgNode); 
		}
	}
	
	// Number of samples
	randomCount = widget->synthesisSamplesCount();
	uniformTriCount = widget->synthesisSamplesCount();

	// Generate synthesis data for each corresponding node
	foreach(Structure::Node * snode, scheduler->activeGraph->nodes)
	{
		Structure::Node * tnode = scheduler->targetGraph->getNode( snode->property["correspond"].toString() );

		//int sampling_method = Synthesizer::Random | Synthesizer::Features;
		int sampling_method = Synthesizer::TriUniform | Synthesizer::Features;
		//int sampling_method = Synthesizer::Features;
		//int sampling_method = Synthesizer::Uniform;
		//int sampling_method = Synthesizer::Remeshing;
		//int sampling_method = Synthesizer::Random | Synthesizer::Features | Synthesizer::TriUniform;

		if(snode->type() == Structure::CURVE)
		{
			Synthesizer::prepareSynthesizeCurve((Structure::Curve*)snode, (Structure::Curve*)tnode, sampling_method);
		}
			
		if(snode->type() == Structure::SHEET)
		{
			Synthesizer::prepareSynthesizeSheet((Structure::Sheet*)snode, (Structure::Sheet*)tnode, sampling_method);
		}
				
		int percent = (double(n) / (numNodes-1) * 100);
		emit( statusBarMessage(QString("Generating data.. [ %1 % ]").arg(percent)) );
		n++;
	}

	// Copy to current displayed graphs
	//foreach(Node* n, scheduler->activeGraph->nodes) Synthesizer::copySynthData( n, graphs.front()->getNode(n->id.split("_").at(0)) );
	//foreach(Node* n, scheduler->targetGraph->nodes) Synthesizer::copySynthData( n, graphs.back()->getNode(n->id.split("_").at(0)) );

	QString timingString = QString("Synthesis data [ %1 ms ]").arg(timer.elapsed());
	qDebug() << timingString;
	emit( statusBarMessage(timingString) );

	scheduler->property["synthDataReady"] = true;

	qApp->restoreOverrideCursor();
}

void topoblend::setStatusBarMessage(QString message)
{
	mainWindow()->setStatusBarMessage(message, 1000);
	drawArea()->updateGL();
}

void topoblend::generateSynthesisData()
{
	if(!blender) return;

	setStatusBarMessage("Generating synthesis samples...");

#ifdef Q_OS_WIN
    QtConcurrent::run( this, &topoblend::genSynData );
#else // OpenMP issue on OSX (Linux too?)
    genSynData();
#endif
}

void topoblend::saveSynthesisData()
{
	if(!blender) return;

	QString foldername = gcoor->sgName() + "_" + gcoor->tgName();
	QDir dir; dir.mkdir(foldername); dir.setCurrent(foldername);

	foreach(Structure::Node * node, scheduler->activeGraph->nodes)
		Synthesizer::saveSynthesisData(node, "[activeGraph]");
	
	foreach(Structure::Node * node, scheduler->targetGraph->nodes)
		Synthesizer::saveSynthesisData(node, "[targetGraph]");

	statusBarMessage("Synth data saved.");

	dir.cdUp();
}

void topoblend::loadSynthesisData()
{
	if(!blender) return;

	drawArea()->camera()->setType(qglviewer::Camera::PERSPECTIVE);

	QString foldername = gcoor->sgName() + "_" + gcoor->tgName();
	QDir dir; dir.setCurrent(foldername);

	foreach(Structure::Node * node, scheduler->activeGraph->nodes)
	{
		Synthesizer::clearSynthData(node); // Force load
		Synthesizer::loadSynthesisData(node, "[activeGraph]");
	}

	foreach(Structure::Node * node, scheduler->targetGraph->nodes)
	{
		Synthesizer::clearSynthData(node);
		Synthesizer::loadSynthesisData(node, "[targetGraph]");
	}

	this->graphs.clear();
	this->graphs.push_back(new Structure::Graph(*scheduler->activeGraph));
	this->graphs.push_back(new Structure::Graph(*scheduler->targetGraph));

	statusBarMessage("Synth data loaded.");

	drawArea()->updateGL();

	dir.cdUp();
}

void topoblend::outputPointCloud()
{
	if(!blender) return;
}

void topoblend::renderAll()
{
	if(!scheduler) return;

	int reconLevel = 7;
	if(scheduler->property.contains("reconLevel")){
		reconLevel = scheduler->property["reconLevel"].toInt();
	}

	setStatusBarMessage(QString("Rendering  requested frames [depth=%1]...").arg(reconLevel));

#ifdef Q_OS_WIN
	QtConcurrent::run( this, &topoblend::doRenderAll );
#else // OpenMP issue on OSX (Linux too?)
	doRenderAll();
#endif

	//doRenderAll();
}

void topoblend::doRenderAll()
{	
	QElapsedTimer timer; timer.start();

	int stepSize = 1;
	int N = scheduler->allGraphs.size();

	if(scheduler->property.contains("renderCount")){
		int renderCount = scheduler->property["renderCount"].toInt();
		if(renderCount > 1)
			stepSize = double(N) / renderCount;
	}

	int reconLevel = 7;
	if(scheduler->property.contains("reconLevel")){
		reconLevel = scheduler->property["reconLevel"].toInt();
	}

	int startPercentage = scheduler->property["renderStartPercentage"].toInt();
	int startID = N * ( (double) startPercentage / 100);

	for(int i = startID; i < N; i += stepSize)
	{
		Structure::Graph currentGraph = *(scheduler->allGraphs[i]);

		if(i > 0) scheduler->allGraphs[i-1]->clearGeometryCache();

		int progress = (double(i) / (N-1)) * 100;
		qDebug() << QString("Rendering sequence [%1 %]").arg(progress);

		if (progress < 10)
			renderGraph( currentGraph, QString("output_0%1").arg(progress), false, reconLevel );
		else
			renderGraph( currentGraph, QString("output_%1").arg(progress), false, reconLevel );
	}

	qDebug() << QString("Sequence rendered [%1 ms]").arg(timer.elapsed());
}

void topoblend::renderCurrent()
{
	if(!scheduler) return;

	QElapsedTimer timer; timer.start();

	int reconLevel = 7;
	if(scheduler->property.contains("reconLevel")){
		reconLevel = scheduler->property["reconLevel"].toInt();
	}

	setStatusBarMessage(QString("Rendering current frame [depth=%1]..").arg(reconLevel));

	Structure::Graph * currentGraph = this->graphs.back();

	QDir dir("");
	dir.setCurrent(QFileDialog::getExistingDirectory());

	// Reconstruct
	QString filename = "currentGraph";
	Structure::Graph lastGraph = *currentGraph;
	renderGraph(lastGraph, filename, false, reconLevel, true);

	// Load and display as mesh in viewer
	if( false )
	{
		SurfaceMesh::SurfaceMeshModel * m = new SurfaceMesh::SurfaceMeshModel(filename + ".obj", "reconMesh");
		m->read( qPrintable(filename+".obj") );
		currentGraph->property["reconMesh"].setValue( m );
	}

	updateDrawArea();

	mainWindow()->setStatusBarMessage(QString("Current graph rendered [%1 ms]").arg(timer.elapsed()));
}

void topoblend::renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph )
{
	graph.clearGeometryCache();
	graph.geometryMorph();

	QStringList generatedFiles, tempFiles;

	foreach(Structure::Node * node, graph.nodes)
	{
		// Skip inactive nodes
		if( node->property["zeroGeometry"].toBool() || node->property["shrunk"].toBool() || 
			!node->property.contains("cached_points")) continue;

		QVector<Vec3f> points = node->property["cached_points"].value< QVector<Vec3f> >();
		QVector<Vec3f> normals = node->property["cached_normals"].value< QVector<Vec3f> >();

		if(!points.size()) continue;
		std::vector<Vec3f> clean_points;
		foreach(Vec3f p, points) clean_points.push_back(p);

		std::vector< Vec3f > finalP, finalN;

		/// Clean up duplicated points
		if( false )
		{
			std::vector<size_t> xrefs;
			weld(clean_points, xrefs, std::hash_Vec3d(), std::equal_to<Vec3f>());

			std::set<int> uniqueIds;
			for(int i = 0; i < (int)points.size(); i++)	uniqueIds.insert(xrefs[i]);

			foreach(int id, uniqueIds)
			{
				finalP.push_back( points[id] );
				finalN.push_back( normals[id] );
			}
		}
		else
		{
			foreach(Vec3d p, points) finalP.push_back(p);
			foreach(Vec3d n, normals) finalN.push_back(n);
		}

		/// Better Estimate Normals
		{
			//int num_nighbours = 10;
			//NormalExtrapolation::ExtrapolateNormals(clean_points, normals, num_nighbours);
		}

		/// Smooth normals
		if( true )
		{
			NanoKdTree tree;
			foreach(Vector3 p, finalP) tree.addPoint(p);
			tree.build();

			for(int i = 0; i < (int)finalP.size(); i++)
			{
				Vec3d newNormal(0);

				int k = 12;

				KDResults matches;
				tree.k_closest(finalP[i], k, matches);
				foreach(KDResultPair match, matches) newNormal += finalN[match.first];
				newNormal /= 12.0;

				finalN[i] = newNormal;
			}
		}

		/// Send for reconstruction:
		if(isOutPointCloud)
		{
			QString xyz_filename = node->id + "_" + filename + ".xyz";
			tempFiles << xyz_filename;
			Synthesizer::writeXYZ( xyz_filename, finalP, finalN );
		}

		QString node_filename = node->id + ".obj";
		generatedFiles << node_filename;
		PoissonRecon::makeFromCloud( pointCloudf(finalP), pointCloudf(finalN), node_filename, reconLevel );

		// Replace with reconstructed geometries
		if( isOutGraph )
		{
			QFileInfo reconFile( node_filename );

			if( reconFile.exists() )
			{
				SurfaceMesh::Model* nodeMesh = new SurfaceMesh::Model;
				nodeMesh->read( qPrintable(node_filename) );

				node->property["mesh"].setValue( nodeMesh );
				node->property["mesh_filename"].setValue( "meshes/" + node_filename );
			}
		}

		// Clean up
		scheduler->cleanUp();
	}

	combineMeshes(generatedFiles, filename + ".obj");

	if( isOutGraph )
	{
		// Find any removable nodes
		QStringList removableNodes;
		foreach(Node * n, graph.nodes)
		{
			int taskType = n->property["taskType"].toInt();
			bool taskReady = n->property["taskIsReady"].toBool();
			bool taskDone = n->property["taskIsDone"].toBool();
			double t = n->property["t"].toDouble();

			// To-do: cut nodes cases
			{
				// Not yet grown
				if( taskType == Task::GROW && !taskReady && t > 0.0 )
					removableNodes << n->id;

				// Shrunk nodes
				if( taskType == Task::SHRINK && taskDone )
					removableNodes << n->id;
			}
		}

		// Remove
		foreach(QString nodeID, removableNodes)
		{
			graph.removeNode( nodeID );
		}

		graph.saveToFile( filename + ".xml" );
	}

	// Clean up
	if(!isOutPointCloud)
	{
		foreach(QString filename, generatedFiles)
			QFile::remove( filename );
	}
}

void topoblend::draftRender()
{
	if(!scheduler) return;

	int stepSize = 1;
	int N = scheduler->allGraphs.size();

	if(scheduler->property.contains("renderCount")){
		int renderCount = scheduler->property["renderCount"].toInt();
		if(renderCount > 1)
			stepSize = double(N) / renderCount;
	}

	if(N < 1) return;

	// Setup camera
	//qglviewer::Camera * camera = drawArea()->camera();

	QString folderPath = QFileDialog::getExistingDirectory();

	for(int i = 0; i < N; i += stepSize)
	{
		graphs.clear();
		this->graphs.push_back( scheduler->allGraphs[i] );
		drawArea()->updateGL();

		// Save snapshot
		QString snapshotFile;
		int progress = (double(i) / (N-1)) * 100;
		snapshotFile.sprintf("draft_%05d.png", progress);
		snapshotFile = folderPath + "/" + snapshotFile;
		drawArea()->saveSnapshot(snapshotFile, true);

		qDebug() << "File saved: " << snapshotFile;
	}
}

void topoblend::reconstructXYZ()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open XYZ File"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("XYZ Files (*.xyz)"));
	if(fileNames.isEmpty()) return;

	foreach(QString filename, fileNames)
	{
		PoissonRecon::makeFromCloudFile(filename, filename + ".off", 7);
	}
}

void topoblend::combineMeshesToOne()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Mesh File"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("OFF Files (*.off)"));
	if(fileNames.isEmpty()) return;

	QString out_filename = QFileDialog::getSaveFileName(0, tr("Save Mesh"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("OBJ Files (*.obj)"));
	if(out_filename.isEmpty()) return;

	combineMeshes(fileNames, out_filename);
}

void topoblend::normalizeAllGraphs()
{
	foreach (Structure::Graph* g, graphs)
	{
		g->moveBottomCenterToOrigin();
		g->normalize();
	}

	drawArea()->updateGL();
}

Q_EXPORT_PLUGIN(topoblend)
