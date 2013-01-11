#include <QElapsedTimer>
#include <QFileDialog>
#include <QDialog>
#include <QStack>
#include <QQueue>

#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"
#include "graph_modify_dialog.h"

// Graph manipulations
#include "DynamicGraph.h"
#include "GraphDistance.h"
#include "TopoBlender.h"
#include "Scheduler.h"

// Graph Correspondence
#include "GraphCorresponder.h"

// Temporary solution
#include "surface_mesh/IO.h"
#include "surface_mesh/IO_off.cpp"

// Temp
TopoBlender * blender = NULL;
Scheduler * scheduler = NULL;

#include "ARAPCurveDeformer.h"
ARAPCurveDeformer * deformer = NULL;
#include "ARAPCurveHandle.h"
ARAPCurveHandle * handle = NULL;

topoblend::topoblend(){
	widget = NULL;
	gcoor = NULL;
	layout = true;
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

		loadModel();
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

	// Points
	glPointSize(10); glColor3d(1,0,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints) glVector3(v); glEnd();
	glPointSize(15); glColor3d(1,1,1); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints) glVector3(v); glEnd();
	glColor3d(0,1,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints2) glVector3(v); glEnd();
	glColor3d(0,0,1); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints3) glVector3(v); glEnd();

	// Lines
	typedef QPair<Vector3,Vector3> MyLine;
	glColor3d(1,0,0); glBegin(GL_LINES); foreach(MyLine l, debugLines) {glVector3(l.first);  glVector3(l.second);} glEnd();
	glColor3d(0,1,0); glBegin(GL_LINES); foreach(MyLine l, debugLines2) {glVector3(l.first); glVector3(l.second);} glEnd();
	glColor3d(0,0,1); glBegin(GL_LINES); foreach(MyLine l, debugLines3) {glVector3(l.first); glVector3(l.second);} glEnd();

	glEnable(GL_LIGHTING);

	glColor3d(1,1,1);
	drawArea()->drawText(40,40, "TopoBlend mode.");

	if(blender) blender->drawDebug();

	// 3D visualization
	glEnable(GL_LIGHTING);

	double deltaX = layout ? drawArea()->sceneRadius() : 0;
	double posX = - deltaX * (graphs.size() - 1) / 2;

	for(int g = 0; g < (int) graphs.size(); g++)
	{
		glPushMatrix();
		glTranslatef(posX, 0, 0);
		graphs[g]->draw();
		glPopMatrix();

		posX += deltaX;
	}

	if(deformer)
	{

	}
}


void topoblend::drawWithNames()
{
	float deltaX = layout ? drawArea()->sceneRadius() : 0;
	float posX = - deltaX * (graphs.size() - 1) / 2;

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
	int selectedID = drawArea()->selectedName();
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

    NURBSRectangle backSheet = NURBSRectangle::createSheet(2,1, Vector3(0,-0.5,2), dU, dV);

	NURBSRectangle seatSheet = NURBSRectangle::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,1,0));
	NURBSRectangle seatSheetInv = NURBSRectangle::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,-1,0));

	NURBSCurve backLeft = NURBSCurve::createCurve(Vector3(-0.9,0,0), Vector3(-0.9,-0.5,1.5));
	NURBSCurve backRight = NURBSCurve::createCurve(Vector3(0.9,0,0), Vector3(0.9,-0.5,1.5));
	NURBSCurve backLeft2 = NURBSCurve::createCurve(Vector3(-0.5,0,0), Vector3(-0.5,-0.5,1.5));
	NURBSCurve backRight2 = NURBSCurve::createCurve(Vector3(0.5,0,0), Vector3(0.5,-0.5,1.5));
	NURBSCurve frontLegLeft = NURBSCurve::createCurve(Vector3(-1,1.75,0), Vector3(-1,1.9,-2));
	NURBSCurve frontLegRight = NURBSCurve::createCurve(Vector3(1,1.75,0), Vector3(1,1.9,-2));
	NURBSCurve backLegLeft = NURBSCurve::createCurve(Vector3(-1,0.25,0), Vector3(-1,0,-2));
	NURBSCurve backLegRight = NURBSCurve::createCurve(Vector3(1,0.25,0), Vector3(1,0,-2));
	NURBSCurve backTop = NURBSCurve::createCurve(Vector3(-0.9,-0.3345,1), Vector3(0.9,-0.3345,1));
	NURBSCurve backBottom = NURBSCurve::createCurve(Vector3(-0.9,-0.171,0.5), Vector3(0.9,-0.171,0.5));
	NURBSCurve legBarLeft = NURBSCurve::createCurve(Vector3(-1,0.126,-1), Vector3(-1,1.825,-1));
	NURBSCurve legBarMiddle = NURBSCurve::createCurve(Vector3(-1,1.825,-1), Vector3(1,1.825,-1));
	NURBSCurve legBarRight = NURBSCurve::createCurve(Vector3(1,0.126,-1), Vector3(1,1.825,-1));
	NURBSCurve extraBar = NURBSCurve::createCurve(Vector3(1,0.079,-1.36), Vector3(1,-0.2,-1.6));

	NURBSCurve swivel1MiddleBar = NURBSCurve::createCurve(Vector3(0,1,0), Vector3(0,1,-2));
	NURBSCurve swivel1Branch1 = NURBSCurve::createCurve(Vector3(0,1,-2), Vector3(1,1.9,-2));
	NURBSCurve swivel1Branch2 = NURBSCurve::createCurve(Vector3(0,1,-2), Vector3(-1,1.9,-2));
	NURBSCurve swivel1Branch3 = NURBSCurve::createCurve(Vector3(0,1,-2), Vector3(1,0,-2));
	NURBSCurve swivel1Branch4 = NURBSCurve::createCurve(Vector3(0,1,-2), Vector3(-1,0,-2));

	NURBSCurve swivel2MiddleBar = NURBSCurve::createCurve(Vector3(0,1,0), Vector3(0,1,-0.5));
	NURBSCurve swivel2Branch1 = NURBSCurve::createCurve(Vector3(0,1,-0.5), Vector3(1,1.9,-2));
	NURBSCurve swivel2Branch2 = NURBSCurve::createCurve(Vector3(0,1,-0.5), Vector3(-1,1.9,-2));
	NURBSCurve swivel2Branch3 = NURBSCurve::createCurve(Vector3(0,1,-0.5), Vector3(1,0,-2));
	NURBSCurve swivel2Branch4 = NURBSCurve::createCurve(Vector3(0,1,-0.5), Vector3(-1,0,-2));

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
    NURBSRectangle centerSheet = NURBSRectangle::createSheet(1,1, Vector3(0,0,0), dU, dV);
    NURBSRectangle leftSheet = NURBSRectangle::createSheet(1,1, Vector3(-3,0,2), dU, dV);
    NURBSRectangle rightSheet = NURBSRectangle::createSheet(1,1, Vector3(3,0,2), dU, dV);
    NURBSRectangle bottomSheet = NURBSRectangle::createSheet(1,1, Vector3(0,0,-2.5), dU, dV);

	NURBSCurve leftCurve = NURBSCurve::createCurve(Vector3(-0.5,0,0.5), Vector3(-2.5, 0, 1.5));
	NURBSCurve rightCurve = NURBSCurve::createCurve(Vector3(0.5,0,0.5), Vector3(2.5, 0, 1.5));
	NURBSCurve bottomCurve = NURBSCurve::createCurve(Vector3(0,0,-0.5), Vector3(0, 0, -2.0));
	NURBSCurve bottomLeftCurve = NURBSCurve::createCurve(Vector3(-2.5, 0, 1.5), Vector3(-0.5,0,-2));
	NURBSCurve bottomRightCurve = NURBSCurve::createCurve(Vector3(2.5, 0, 1.5), Vector3(0.5,0,-2));

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
	QBox3D bigbox = graphs.front()->bbox();
	for(int i = 0; i < (int)graphs.size(); i++)
		bigbox.unite( graphs[i]->bbox() );

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

	foreach(QString file, fileNames)
		graphs.push_back( new Structure::Graph ( file ) );

	setSceneBounds();
}

void topoblend::saveModel()
{
	if (graphs.size() < 1){
		qDebug() << "Please load a graph.";
		return;
	}

	QString filename = QFileDialog::getSaveFileName(0, tr("Save Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

	graphs.back()->saveToFile(filename);
}

void topoblend::modifyModel()
{
	if (graphs.size() < 1){
		qDebug() << "Please load a graph";
		return;
	}
	
    GraphModifyDialog modifyDialog(graphs.back());
	drawArea()->connect(&modifyDialog, SIGNAL(updateView()), SLOT(updateGL()));
	modifyDialog.exec();
}

bool topoblend::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	QElapsedTimer timer; timer.start();

	if(event->key() == Qt::Key_Space)
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

			SurfaceMeshModel * m = new SurfaceMeshModel( QString("Voxel_%1.obj").arg(g), QString("Voxel_%1").arg(g) );
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

	if(event->key() == Qt::Key_R)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			qDebug() << "Root (by valence) = " << graphs[g]->rootByValence()->id;
			qDebug() << "Root (by size) = " << graphs[g]->rootBySize()->id;
		}
		used = true;
	}

	if(event->key() == Qt::Key_O)
	{
		if(graphs.size() < 1) return true;

		Structure::Node * n = graphs.front()->nodes.front();
		std::vector<Vec3d> orgCtrlPnts = n->controlPoints();

		deformer = new ARAPCurveDeformer( orgCtrlPnts, orgCtrlPnts.size() * 0.25 );

		deformer->SetAnchor( orgCtrlPnts.size() - 1 );		// Last point as anchor
		deformer->UpdateControl( 0, orgCtrlPnts.front());	// First point moves

		qDebug() << "Curve deformation performed!"; 

		handle = new ARAPCurveHandle(orgCtrlPnts.front(), 0.0);
		drawArea()->setManipulatedFrame( handle );
		this->connect(handle, SIGNAL(manipulated()), SLOT(experimentSlot()));

		used = true;
	}

	drawArea()->updateGL();

	return used;
}

void topoblend::experimentSlot()
{
	Structure::Node * n = graphs.front()->nodes.front();

	qglviewer::Vec v = handle->position();
	deformer->points[0] = Vec3d(v[0],v[1],v[2]);

	deformer->Deform(3);
	n->setControlPoints( deformer->points );
}

void topoblend::doBlend()
{
	if (graphs.size() < 2)
	{
		qDebug() << "Please load at least two graphs.";
		return;
	}

	Structure::Graph * source = graphs.front();
	Structure::Graph * target = graphs.back();

	if(scheduler) scheduler->disconnect(this);

	scheduler = new Scheduler();

    blender = new TopoBlender( source, target, corresponder(), scheduler );

	this->connect(scheduler, SIGNAL(activeGraphChanged( Structure::Graph* )), SLOT(updateActiveGraph( Structure::Graph* )));
	//graphs.push_back( blender->active );

	//Structure::Graph * blendedGraph = blender->blend();

	// Set options
	//blender->params["NUM_STEPS"] = this->params["NUM_STEPS"];
	//blender->params["materialize"] = this->params["materialize"];
	//blender->materializeInBetween( blendedGraph, 0, source );
	//graphs.push_back( blendedGraph );

	setSceneBounds();
}

void topoblend::experiment1()
{

}

void topoblend::clearGraphs()
{
	qDeleteAll(graphs);
	graphs.clear();

	blender = NULL;

	drawArea()->updateGL();

	delete gcoor;
	gcoor = NULL;
}

void topoblend::currentExperiment()
{
    Vector3 dU = Vector3(1,0,0);
    Vector3 dV = Vector3(0,0,1);

    NURBSRectangle sheetA = NURBSRectangle::createSheet(1.75,2, Vector3(0,0.25,0), dU, dV);
	NURBSRectangle sheetB = NURBSRectangle::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,1,0));

	sheetA.bend(-0.7);
	sheetB.bend(0.2);
	sheetB.bend(0.1,1);

	Structure::Graph * graph = new Structure::Graph();

	graph->addNode( new Structure::Sheet( sheetA, "SheetA" ) );
	graph->addNode( new Structure::Sheet( sheetB, "SheetB" ) );

	graph->addEdge( graph->getNode("SheetA"), graph->getNode("SheetB") );

	graphs.push_back( graph );
	setSceneBounds();

	// Test graph distance on a single node
	//GraphDistance * gd = new GraphDistance( graph->nodes.front() );
	//gd->computeDistances( Vector3(0,0.25,0) );
	//graphs.back()->misc["distance"] = gd;

	qDebug() << "Done";

    //Structure::Graph * blendedGraph = blender->blend();

    // Set options
    //blender->params["NUM_STEPS"] = this->params["NUM_STEPS"];
    //blender->params["materialize"] = this->params["materialize"];
    //blender->materializeInBetween( blendedGraph, 0, source );
    //graphs.push_back( blendedGraph );
    //setSceneBounds();
}

void topoblend::updateDrawArea()
{
	drawArea()->updateGL();
}

GraphCorresponder* topoblend::corresponder()
{
	if (!gcoor)
	{
		if (graphs.size() < 2)
		{
			qDebug() << "Please load at least two graphs.";
			return NULL;
		}

		Structure::Graph *sg = graphs[0];
		Structure::Graph *tg = graphs[1];

		gcoor = new GraphCorresponder(sg, tg);
	}

	return gcoor;
}


void topoblend::testPoint2PointCorrespondences()
{
	// Create graphs
	Structure::Graph *sg = new Structure::Graph();
	Structure::Graph *tg = new Structure::Graph();
	Structure::Graph *newG = new Structure::Graph();

	graphs.push_back(sg);
	graphs.push_back(tg);
	graphs.push_back(newG);

	// Create one sheet for each graph
	NURBSRectangle sheet1 = NURBSRectangle::createSheet(2,1, Vector3(0,0,0), Vector3(1,0,0), Vector3(0,1,0));
	Structure::Sheet *sSheet = new Structure::Sheet(sheet1, "sheet");
	sg->addNode(sSheet);

	NURBSRectangle sheet2 = NURBSRectangle::createSheet(2,1.5, Vector3(0,0,0), Vector3(-1,0,0), Vector3(0,-1,0));
	Structure::Sheet *tSheet = new Structure::Sheet(sheet2, "sheet");
	tg->addNode(tSheet);

	Structure::Sheet *newSheet = new Structure::Sheet(*tSheet);
	newG->addNode(newSheet);

	NURBSCurve curve = NURBSCurve::createCurve(Vector3(0.3, 0.4, 0), Vector3(0.3, 0.6, 1));
	sg->addNode(new Structure::Curve(curve, "curve"));
	sg->addEdge("sheet", "curve");
	tg->addNode(new Structure::Curve(curve, "curve"));
	tg->addEdge("sheet", "curve");
	newG->addNode(new Structure::Curve(curve, "curve"));
	newG->addEdge("sheet", "curve");

	// Realign two sheets
	if (corresponder())
		corresponder()->correspondTwoSheets(sSheet, tSheet);

	//// Create one curve for each graph
	//NURBSCurve curve1 = NURBSCurve::createCurve(Vector3(0,-1,0), Vector3(0,1,0));
	//Structure::Curve *sCurve = new Structure::Curve(curve1, "curve");
	//sg->addNode(sCurve);

	//NURBSCurve curve2 = NURBSCurve::createCurve(Vector3(0, 1, 0), Vector3(0, -1, 0));
	//Structure::Curve *tCurve = new Structure::Curve(curve2, "curve");
	//tg->addNode(tCurve);

	//Structure::Curve *newCurve = new Structure::Curve(*tCurve);
	//newG->addNode(newCurve);

	//NURBSCurve curve3 = NURBSCurve::createCurve(Vector3(-0.1, 0.7, 0), Vector3(0.5, 0.7, 0));
	//sg->addNode(new Structure::Curve(curve3, "hcurve"));
	//sg->addEdge("curve", "hcurve");
	//tg->addNode(new Structure::Curve(curve3, "hcurve"));
	//tg->addEdge("curve", "hcurve");
	//newG->addNode(new Structure::Curve(curve3, "hcurve"));
	//newG->addEdge("curve", "hcurve");

	//if (corresponder())
	//	corresponder()->correspondTwoCurves(sCurve, tCurve);

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
	graphs.clear();
	this->graphs.push_back(newActiveGraph);
	drawArea()->updateGL();
}


Q_EXPORT_PLUGIN(topoblend)
