#include <QElapsedTimer>
#include <QFileDialog>

#include "topo-blend.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"

#include "topo/TopoBlender.h"

#include "topo/DynamicGraph.h"

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
		graphs[g].draw2D(150,100);
		drawArea()->stopScreenCoordinatesSystem();
	}

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
	graphs.push_back( Structure::Graph("simple_model1.xml") );
	graphs.push_back( Structure::Graph("simple_model2.xml") );

	DynamicGraph g(&graphs.front());
	DynamicGraph g2(&graphs.back());

	g.printState();
	g2.printState();

	GraphState diff = g.difference( g2.State() );

	diff.print();

	setSceneBounds();
}


Q_EXPORT_PLUGIN(topoblend)
