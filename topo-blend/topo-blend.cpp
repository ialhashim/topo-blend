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
#include "GraphDistance.h"
#include "TopoBlender.h"
#include "Scheduler.h"

// Graph Correspondence
#include "GraphCorresponder.h"

// Temp
TopoBlender * blender = NULL;
Scheduler * scheduler = NULL;

#include "ARAPCurveDeformer.h"
ARAPCurveDeformer * deformer = NULL;
#include "ARAPCurveHandle.h"
ARAPCurveHandle * handle = NULL;

#include "Synthesizer.h"
#include "normal_extrapolation.h"

#include "poissonrecon.h"

VectorSoup vs1,vs2;

Q_DECLARE_METATYPE(Vec3d)

#include "RMF.h"
Q_DECLARE_METATYPE(RMF::Frame)
Q_DECLARE_METATYPE(std::vector<RMF::Frame>)

double boundX = -DBL_MAX;

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

	glColor3d(1,1,1);
	drawArea()->drawText(40,40, "TopoBlend mode.");

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

		// Place and draw graph
		glPushMatrix();
		glTranslatef(posX, 0, 0);
        graphs[g]->draw();
		glPopMatrix();

		posX += deltaX;

		if(graphs[g]->property.contains("selectedSample"))
		{
			glPointSize(20);
			glColor3d(1,1,0);
			glDisable(GL_LIGHTING);
			glBegin(GL_POINTS);
			glVector3( graphs[g]->property["selectedSample"].value<Vec3d>() );
			glEnd();
			glEnable(GL_LIGHTING);

			Structure::Node * node = graphs[g]->getNode(graphs[g]->property["selectedNode"].toString());
			if(node->property.contains("rmf_frames")){
				std::vector<RMF::Frame> U = node->property["rmf_frames"].value< std::vector<RMF::Frame> >();
				FrameSoup * fs = new FrameSoup(0.005f);
				foreach(RMF::Frame f, U){
					fs->addFrame(f.r, f.s, f.t, f.center);
				}
				fs->draw();
			}
		}
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
    Q_UNUSED(point);
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
		Structure::Graph * g = new Structure::Graph ( file );

		if( widget->isModifyModelOnLoad() )
		{
			g->normalize();
			g->moveBottomCenterToOrigin();
		}

		graphs.push_back( g );
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

	if(event->key() == Qt::Key_T)
	{
		qglviewer::Vec q = drawArea()->camera()->revolveAroundPoint();
		Vec3d p(q[0],q[1],q[2]);

		double minDist = DBL_MAX;
		QString sample_details = "";

		foreach(Structure::Graph * g, graphs)
		{
			foreach(Structure::Node * node, scheduler->activeGraph->nodes)
			{
				if(node->property.contains("cached_points"))
				{
					QVector<Vec3d> pnts = node->property["cached_points"].value< QVector<Vec3d> >();
					QVector<ParameterCoord> samples = node->property["samples"].value< QVector<ParameterCoord> >();
					QVector<double> offsets = node->property["offsets"].value< QVector<double> >();

					for(int i = 0; i < (int)pnts.size(); i++)
					{
						double dist = (pnts[i] - p).norm();
						if(dist < minDist){
							ParameterCoord s = samples[i];
							sample_details = QString("[%5] u= %1  v= %2  theta= %3  psi= %4 offset = %6").arg(s.u
								).arg(s.v).arg(s.theta).arg(s.psi).arg(node->id).arg(offsets[i]);
							minDist = dist;
							g->property["selectedSample"].setValue(pnts[i]);
							g->property["selectedNode"].setValue(node->id);
						}
					}
				}
			}
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

	if(event->key() == Qt::Key_L)
	{
		Structure::Graph * g = graphs.front();

		QVector< QVector<Node*> > parts = g->split("Central");

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

	QElapsedTimer timer; timer.start();

	scheduler = new Scheduler();
    blender = new TopoBlender( source, target, corresponder(), scheduler );

	qDebug() << QString("Created TopoBlender and tasks in [ %1 ms ]").arg(timer.elapsed()  );

	this->connect(scheduler, SIGNAL(activeGraphChanged( Structure::Graph* )), SLOT(updateActiveGraph( Structure::Graph* )));
	
	this->connect(scheduler, SIGNAL(renderAll()), SLOT(renderAll()));
	this->connect(scheduler, SIGNAL(renderCurrent()), SLOT(renderCurrent()));
	this->connect(scheduler, SIGNAL(draftRender()), SLOT(draftRender()));

	//this->graphs.clear();
	//this->graphs.push_back(scheduler->activeGraph);
	//this->graphs.push_back(scheduler->targetGraph);

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

	debugPoints.clear();
	debugPoints2.clear();
}

void topoblend::currentExperiment()
{

}

void topoblend::updateDrawArea()
{
	setSceneBounds();

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
    NURBSRectangled sheet1 = NURBSRectangled::createSheet(2,1, Vector3(0,0,0), Vector3(1,0,0), Vector3(0,1,0));
	Structure::Sheet *sSheet = new Structure::Sheet(sheet1, "sheet");
	sg->addNode(sSheet);

    NURBSRectangled sheet2 = NURBSRectangled::createSheet(2,1.5, Vector3(0,0,0), Vector3(-1,0,0), Vector3(0,-1,0));
	Structure::Sheet *tSheet = new Structure::Sheet(sheet2, "sheet");
	tg->addNode(tSheet);

	Structure::Sheet *newSheet = new Structure::Sheet(*tSheet);
	newG->addNode(newSheet);

    NURBSCurved curve = NURBSCurved::createCurve(Vector3(0.3, 0.4, 0), Vector3(0.3, 0.6, 1));
	sg->addNode(new Structure::Curve(curve, "curve"));
	sg->addEdge("sheet", "curve");
	tg->addNode(new Structure::Curve(curve, "curve"));
	tg->addEdge("sheet", "curve");
	newG->addNode(new Structure::Curve(curve, "curve"));
	newG->addEdge("sheet", "curve");

	// Realign two sheets
	if (corresponder())
		corresponder()->correspondTwoSheets(sSheet, tSheet, tg);

	//// Create one curve for each graph
    //NURBSCurved curve1 = NURBSCurved::createCurve(Vector3(0,-1,0), Vector3(0,1,0));
	//Structure::Curve *sCurve = new Structure::Curve(curve1, "curve");
	//sg->addNode(sCurve);

    //NURBSCurved curve2 = NURBSCurved::createCurve(Vector3(0, 1, 0), Vector3(0, -1, 0));
	//Structure::Curve *tCurve = new Structure::Curve(curve2, "curve");
	//tg->addNode(tCurve);

	//Structure::Curve *newCurve = new Structure::Curve(*tCurve);
	//newG->addNode(newCurve);

    //NURBSCurved Curve = NURBSCurved::createCurve(Vector3(-0.1, 0.7, 0), Vector3(0.5, 0.7, 0));
	//sg->addNode(new Structure::Curve(Curve, "hcurve"));
	//sg->addEdge("curve", "hcurve");
	//tg->addNode(new Structure::Curve(Curve, "hcurve"));
	//tg->addEdge("curve", "hcurve");
	//newG->addNode(new Structure::Curve(Curve, "hcurve"));
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
	foreach(Structure::Node * node, scheduler->activeGraph->nodes)
	{
		if(node->property.contains("correspond"))
		{
			QString nodeID = node->id;
			QString tnodeID = node->property["correspond"].toString();
			Structure::Node * tgNode = scheduler->targetGraph->getNode(tnodeID);

			//int sampling_method = Synthesizer::Random | Synthesizer::Features;
			int sampling_method = Synthesizer::TriUniform | Synthesizer::Features;
			//int sampling_method = Synthesizer::Features;
			//int sampling_method = Synthesizer::Uniform;
			//int sampling_method = Synthesizer::Remeshing;
			//int sampling_method = Synthesizer::Random | Synthesizer::Features | Synthesizer::TriUniform;

			if(node->type() == Structure::CURVE)
			{
				Synthesizer::prepareSynthesizeCurve((Structure::Curve*)node, (Structure::Curve*)tgNode, sampling_method);
			}

			if(node->type() == Structure::SHEET)
			{
				Synthesizer::prepareSynthesizeSheet((Structure::Sheet*)node, (Structure::Sheet*)tgNode, sampling_method);
			}

			// Copy samples to clones
			if(nodeID.contains("_"))
			{
				QString id = nodeID.split("_").at(0);
				foreach(Structure::Node * other_node, scheduler->activeGraph->nodes)
				{
					if( other_node->id != nodeID && other_node->id.contains(id) )
						Synthesizer::copySynthData( node, other_node );
				}
			}
			if(tnodeID.contains("_"))
			{
				QString id = tnodeID.split("_").at(0);
				foreach(Structure::Node * other_node, scheduler->targetGraph->nodes)
				{
					if( other_node->id != tnodeID && other_node->id.contains(id) )
						Synthesizer::copySynthData( tgNode, other_node );
				}
			}
		}

		// Show results on current source [front] and target [back] graphs
		{
			QString nodeID = node->id;
			QString tnodeID = node->property["correspond"].toString();
			Structure::Node * tnode = scheduler->targetGraph->getNode(tnodeID);

			if(!nodeID.contains("_null")) Synthesizer::copySynthData(node, graphs.front()->getNode(nodeID.split("_").at(0)));
			if(!tnodeID.contains("_null")) Synthesizer::copySynthData(tnode, graphs.back()->getNode(tnodeID.split("_").at(0)));
		}
		
		int percent = (double(n) / (numNodes-1) * 100);
		emit( statusBarMessage(QString("Generating data.. [ %1 % ]").arg(percent)) );
		n++;
	}

	QString timingString = QString("Synthesis data [ %1 ms ]").arg(timer.elapsed());
	qDebug() << timingString;
	emit( statusBarMessage(timingString) );

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

	QtConcurrent::run( this, &topoblend::genSynData );
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

	statusBarMessage("Synth data loaded.");

	drawArea()->updateGL();

	dir.cdUp();
}

void topoblend::outputPointCloud()
{
	if(!blender) return;

	foreach(Structure::Node * n, blender->active->nodes)
	{
		if(n->property.contains("correspond"))
		{
			QString nodeID = n->id;
			QString tnodeID = n->property["correspond"].toString();

			//Synthesizer::writeXYZ();
			QVector<Vec3d> points, normals;

			if(!n->property.contains("cached_points"))
			{
				// Without blending!
				if(n->type() == Structure::CURVE){
					Structure::Curve * curve = (Structure::Curve *)n;
					Synthesizer::blendGeometryCurves(curve,curve,0,points,normals);
				}
				if(n->type() == Structure::SHEET){
					Structure::Sheet * sheet = (Structure::Sheet *)n;
					Synthesizer::blendGeometrySheets(sheet,sheet,0,points,normals);
				}

				n->property["cached_points"].setValue(points);
			}
			else
			{
				points = n->property["cached_points"].value< QVector<Vec3d> >();
			}

			// Estimate normals
			if(points.size())
			{
				int num_nighbours = 16;
				normals.clear();

				std::vector<Vec3d> clean_points;
				foreach(Vec3d p, points) clean_points.push_back(p);

				std::vector<size_t> xrefs;
				weld(clean_points, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());
	
				NormalExtrapolation::ExtrapolateNormals(clean_points, normals, num_nighbours);

				QString xyz_filename = n->id + ".xyz";
				Synthesizer::writeXYZ(xyz_filename, points, normals);

				PoissonRecon::makeFromCloudFile(xyz_filename, xyz_filename + ".off");
			}
		}
	}
}

void topoblend::renderAll()
{
	if(!scheduler) return;

	for(int i = 0; i < scheduler->allGraphs.size(); i++)
	{
		renderGraph( scheduler->allGraphs[i], QString("output_%1.off").arg(i) );
	}
}

void topoblend::renderCurrent()
{
	if(!scheduler) return;
	Structure::Graph * lastGraph = this->graphs.back();
	renderGraph(lastGraph, "currentGraph.off");
}

void topoblend::renderGraph( Structure::Graph * graph, QString filename )
{
	foreach(Structure::Node * node, graph->nodes)
	{
		// Skip inactive nodes
		if(	node->property["toGrow"].toBool() ||
			node->property["shrunk"].toBool() || 
			!node->property.contains("cached_points")) continue;

		QVector<Vec3d> points = node->property["cached_points"].value< QVector<Vec3d> >();
		QVector<Vec3d> normals = node->property["cached_normals"].value< QVector<Vec3d> >();

		if(!points.size()) continue;
		std::vector<Vec3d> clean_points;
		foreach(Vec3d p, points) clean_points.push_back(p);
			 
		// Clean up and compute normals
		int num_nighbours = 10;
		std::vector<size_t> xrefs;
		weld(clean_points, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());

		//NormalExtrapolation::ExtrapolateNormals(clean_points, normals, num_nighbours);

		// Send to reconstruction
		std::vector< std::vector<float> > finalP, finalN;

		foreach(Vec3d p, clean_points){
			std::vector<float> point(3, 0);
			point[0] = p[0];
			point[1] = p[1];
			point[2] = p[2];
			finalP.push_back(point);
		}

		foreach(Vec3d n, normals){
			std::vector<float> normal(3, 0);
			normal[0] = n[0];
			normal[1] = n[1];
			normal[2] = n[2];
			finalN.push_back(normal);
		}

		QString node_filename = node->id + "_" + filename;

		Synthesizer::writeXYZ(node_filename, points, normals);
		//PoissonRecon::makeFromCloud(finalP, finalN, node_filename);
	}
}

void topoblend::draftRender()
{
	if(!scheduler) return;

	int N = scheduler->allGraphs.size();
	if(N < 1) return;

	// Setup camera
	//qglviewer::Camera * camera = drawArea()->camera();

	QString folderPath = QFileDialog::getExistingDirectory();

	for(int i = 0; i < N; i++)
	{
		graphs.clear();
		this->graphs.push_back( scheduler->allGraphs[i] );
		drawArea()->updateGL();

		// Save snapshot
		QString snapshotFile;
		snapshotFile.sprintf("draft_%05d.png", i);
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

Q_EXPORT_PLUGIN(topoblend)
