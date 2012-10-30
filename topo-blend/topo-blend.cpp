#include <QElapsedTimer>
#include <QFileDialog>
#include <QStack>
#include <QQueue>

#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"

#include "topo/TopoBlender.h"

#include "topo/DynamicGraph.h"

// Temporary solution
#include "surface_mesh/IO.h"
#include "surface_mesh/IO_off.cpp"

topoblend::topoblend(){
    widget = NULL;
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
    }
}

void topoblend::decorate()
{
	// 3D visualization
    for(int g = 0; g < (int) graphs.size(); g++)
	{
        graphs[g].draw();
	}

	// 2D view
	//glDisable(GL_LIGHTING);
	for(int g = 0; g < (int) graphs.size(); g++)
	{
		if(graphs[g].edges.size() < 2) continue;

		drawArea()->startScreenCoordinatesSystem();
		graphs[g].draw2D(150,150);
		drawArea()->stopScreenCoordinatesSystem();
	}

	// DEBUG:
	glDisable(GL_LIGHTING);

	// Points
	glColor3d(1,0,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints) glVector3(v); glEnd();
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
}

void topoblend::generateTwoChairModels()
{
	QElapsedTimer assembleTimer; assembleTimer.start(); 

    Structure::Graph chair1, chair2;

    NURBSRectangle backSheet = NURBSRectangle::createSheet(2,1, Vector3(0,-0.5,2));
	NURBSRectangle seatSheet = NURBSRectangle::createSheet(2,2, Vector3(0,1,0), Vector3(1,0,0), Vector3(0,1,0));
	NURBSCurve backLeft = NURBSCurve::createCurve(Vector3(-0.9,0,0), Vector3(-0.9,-0.5,1.5));
	NURBSCurve backRight = NURBSCurve::createCurve(Vector3(0.9,0,0), Vector3(0.9,-0.5,1.5));
	NURBSCurve backLeft2 = NURBSCurve::createCurve(Vector3(-0.5,0,0), Vector3(-0.5,-0.5,1.5));
	NURBSCurve backRight2 = NURBSCurve::createCurve(Vector3(0.5,0,0), Vector3(0.5,-0.5,1.5));
	NURBSCurve frontLegLeft = NURBSCurve::createCurve(Vector3(-1,1.75,0), Vector3(-1,1.9,-2));
	NURBSCurve frontLegRight = NURBSCurve::createCurve(Vector3(1,1.75,0), Vector3(1,1.9,-2));
	NURBSCurve backLegLeft = NURBSCurve::createCurve(Vector3(-1,0.25,0), Vector3(-1,0,-2));
	NURBSCurve backLegRight = NURBSCurve::createCurve(Vector3(1,0.25,0), Vector3(1,0,-2));

	// Chair 1
    chair1.addNode( new Structure::Sheet(backSheet, "BackSheet") );
	chair1.addNode( new Structure::Curve(backLeft, "BackLeft") );
	chair1.addNode( new Structure::Curve(backRight, "BackRight") );
	chair1.addNode( new Structure::Sheet(seatSheet, "SeatSheet") );
	chair1.addNode( new Structure::Curve(frontLegLeft, "FrontLegLeft") );
	chair1.addNode( new Structure::Curve(frontLegRight, "FrontLegRight") );
	chair1.addNode( new Structure::Curve(backLegLeft, "BackLegLeft") );
	chair1.addNode( new Structure::Curve(backLegRight, "BackLegRight") );

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

	// Add edges chair 1
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("BackSheet"), chair1.getNode("BackRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("FrontLegRight") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegLeft") );
	chair1.addEdge( chair1.getNode("SeatSheet"), chair1.getNode("BackLegRight") );

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

	// Save to file
	chair1.saveToFile("chair1.xml");
	chair2.saveToFile("chair2.xml");
	
	//graphs.push_back(chair1);
	//graphs.push_back(chair2);
	
	qDebug() << "Chairs structure graphs construction " << assembleTimer.elapsed() << " ms.";

	setSceneBounds();
}

void topoblend::generateTwoSimpleModels()
{
	QElapsedTimer assembleTimer; assembleTimer.start(); 

	Structure::Graph model1, model2;

	// Geometry
	NURBSRectangle centerSheet = NURBSRectangle::createSheet(1,1, Vector3(0,0,0));
	NURBSRectangle leftSheet = NURBSRectangle::createSheet(1,1, Vector3(-3,0,2));
	NURBSRectangle rightSheet = NURBSRectangle::createSheet(1,1, Vector3(3,0,2));
	NURBSRectangle bottomSheet = NURBSRectangle::createSheet(1,1, Vector3(0,0,-2.5));

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
	QBox3D bigbox = graphs.front().bbox();
	for(int i = 0; i < (int)graphs.size(); i++)
		bigbox.unite( graphs[i].bbox() );

	Vector3 a = bigbox.minimum();
	Vector3 b = bigbox.maximum();
	drawArea()->setSceneBoundingBox(qglviewer::Vec(a.x(), a.y(), a.z()), qglviewer::Vec(b.x(), b.y(), b.z()));

	drawArea()->updateGL();
}

void topoblend::loadModel()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

	foreach(QString file, fileNames)
		graphs.push_back( Structure::Graph ( file ) );

	setSceneBounds();
}

void topoblend::doBlend()
{
    Structure::TopoBlender blender( &graphs.front(), &graphs.last() );

    graphs.push_back( blender.blend() );

	setSceneBounds();
}

bool topoblend::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	QElapsedTimer timer; timer.start();

	if(event->key() == Qt::Key_Space)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			graphs[g].materialize(0);
		}

		qDebug() << "Materialized graphs (voxel only) " << timer.elapsed() << " ms";

		used = true;
	}

	if(event->key() == Qt::Key_E)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			graphs[g].cached_mesh.clear();

			SurfaceMeshModel * m = new SurfaceMeshModel( QString("Voxel_%1.obj").arg(g), QString("Voxel_%1").arg(g) );
			graphs[g].materialize(m);
			DynamicVoxel::MeanCurvatureFlow(m, 0.05);
			//DynamicVoxel::LaplacianSmoothing(m);

			document()->pushBusy();
			document()->addModel(m);
			document()->popBusy();
		}

		qDebug() << "Materialized graphs, generated smooth mesh " << timer.elapsed() << " ms";

		used = true;
	}

	if(event->key() == Qt::Key_M)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
			graphs[g].printAdjacency();
		used = true;
	}

	if(event->key() == Qt::Key_R)
	{
		for(int g = 0; g < (int) graphs.size(); g++)
		{
			qDebug() << "Root (by valence) = " << graphs[g].rootByValence()->id;
			qDebug() << "Root (by size) = " << graphs[g].rootBySize()->id;
		}
		used = true;
	}

	drawArea()->updateGL();

	return used;
}

void topoblend::experiment1()
{
	Structure::Graph * g1 = new Structure::Graph("simple_model1.xml");
	Structure::Graph * g2 = new Structure::Graph("simple_model2.xml");

	DynamicGraph source( g1 );
	DynamicGraph target( g2 );

	// Print states
	source.printState();
	target.printState();
	GraphState diff = source.difference( target.State() );
	diff.print();

	// Solve for nodes and edges discrepancy
	QVector<DynamicGraph> candidates;
	foreach( DynamicGraph gn, source.candidateNodes( target ) )
	{
		foreach( DynamicGraph ge, gn.candidateEdges( target )  )
		{
			// Consider corresponding in terms of [node count] + [edge count] + [valences]
			if( ge.sameValences(target) ) 
				candidates.push_back(ge);
		}
	}

	// Stats
	qDebug() << "Candidates solutions found = " << candidates.size();

	// At this point, corresponding graphs are still ambiguous 
	// when different parts have same connections.
	//
	// Simplest approach is to try out and evaluate all valid combinations.

	// For this test we pick a candidate and work on it
	// g2 = original target
	// g3 = new target

	Structure::Graph g3 = *candidates[3].toStructureGraph();

	QVector<Vector3*> cpoint;
	QVector<Vector3> deltas;
	QVector<Structure::Curve*> curves;

	foreach(Structure::Link e, g3.edges)
	{
		Structure::Node * n1 = e.n1;
		Structure::Node * n2 = e.n2;

		Vector3 pos1(0),pos2(0);

		n1->get(Vector3(e.coord[0][0], e.coord[0][1], 0), pos1);
		n2->get(Vector3(e.coord[1][0], e.coord[1][1], 0), pos2);

		double dist = (pos2-pos1).norm();

		if(dist < 1e-6) continue;

		// 1) Find end closest to any of two nodes		
		Structure::Node * nn = (n1->type() == Structure::CURVE) ? n1: n2;
		Structure::Curve * curve = (Structure::Curve *)nn;

		// Find closest control point
		double minDist = DBL_MAX;
		int idx = 0;
		for(int i = 0; i < (int)curve->curve.mCtrlPoint.size(); i++)
		{
			double dist = (pos1 - curve->curve.mCtrlPoint[i]).norm();

			if(dist < minDist){
				minDist = dist;
				idx = i;
			}
		}
		
		// Compute trajectory to closet on other node
		cpoint.push_back(&(curve->curve.mCtrlPoint[idx]));
		deltas.push_back(pos1-pos2);
		curves.push_back(curve);
	}

	// Generate meshes
	SurfaceMeshModel m;
	g2->materialize(&m);
	DynamicVoxel::MeanCurvatureFlow(&m, 0.05);
	write_off(m, "sequence_.off");

	int NUM_STEPS = 10;
	int NUM_SMOOTH_ITR = 3;

	for(int i = 0; i < cpoint.size(); i++)
	{
		Vector3 delta = deltas[i] / NUM_STEPS;

		for(int s = 0; s < NUM_STEPS; s++)
		{
			Vector3 & cp = *cpoint[i];

			cp += delta;

			// Laplacian smoothing
			for(int itr = 0; itr < NUM_SMOOTH_ITR; itr++)
			{
				QVector<Vector3> newCtrlPnts;
				for (int j = 1; j < (int)curves[i]->curve.mCtrlPoint.size() - 1; j++)
				{
					newCtrlPnts.push_back( (curves[i]->curve.mCtrlPoint[j-1] + curves[i]->curve.mCtrlPoint[j+1]) / 2.0 );
				}
				for (int j = 1; j < curves[i]->curve.mCtrlPoint.size() - 1; j++)
				{
					curves[i]->curve.mCtrlPoint[j] = newCtrlPnts[j-1];
				}
			}

			SurfaceMeshModel meshStep;
			g3.materialize(&meshStep);
			//DynamicVoxel::MeanCurvatureFlow(&meshStep, 0.05);
			DynamicVoxel::LaplacianSmoothing(&meshStep);
			DynamicVoxel::LaplacianSmoothing(&meshStep);

			// output step
			QString fileName = QString("sequence_%1.off").arg((i * NUM_STEPS) + s);
            meshStep.triangulate();
			write_off(meshStep, qPrintable(fileName));
		}
	}

	setSceneBounds();
}

Q_EXPORT_PLUGIN(topoblend)
