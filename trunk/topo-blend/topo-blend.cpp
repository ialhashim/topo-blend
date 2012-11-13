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

	Structure::Graph chair1, chair2, chair3;

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
	NURBSCurve backTop = NURBSCurve::createCurve(Vector3(-0.9,-0.3345,1), Vector3(0.9,-0.3345,1));
	NURBSCurve backBottom = NURBSCurve::createCurve(Vector3(-0.9,-0.171,0.5), Vector3(0.9,-0.171,0.5));

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

	// Save to file
	chair1.saveToFile("chair1.xml");
	chair2.saveToFile("chair2.xml");
	chair3.saveToFile("chair3.xml");

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
	//Structure::Graph * g1 = new Structure::Graph("simple_model1.xml");
	//Structure::Graph * g2 = new Structure::Graph("simple_model2.xml");

	Structure::Graph * g1 = new Structure::Graph("chair1.xml");
	Structure::Graph * g2 = new Structure::Graph("chair2.xml");

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

	// Correspondence
	QMap< double, int > scores;
	double minScore = DBL_MAX;
	
	int N = candidates.size();

	for(int i = 0; i < N; i++)
	{
		double score = 0;
		target.correspondence( candidates[i], score );

		minScore = qMin(score, minScore);
		scores[ score ] = i; 
	}

	DynamicGraph & best = candidates[ scores[ minScore ] ];

	Structure::Graph g3 = *best.toStructureGraph( target );

	QVector<int> cpointIdx;
	QVector<Vector3> deltas;
	QVector<Structure::Curve*> curves;

	for(int ei = 0; ei < g3.edges.size(); ei++)
	{
		Structure::Link & e = g3.edges[ei];

		Structure::Node * n1 = e.n1;
		Structure::Node * n2 = e.n2;

		Vec2d c1 = e.coord[0];
		Vec2d c2 = e.coord[1];

		Vector3 pos1(0), pos2(0);
		n1->get(coord3(c1), pos1);
		n2->get(coord3(c2), pos2);

		double dist = (pos2 - pos1).norm();

		// Check disconnected edge
		if(dist < 1e-6) continue;

		// Get the curve in this link
		Structure::Curve * curve = g3.getCurve(&e);

		Vector3 fixed = n1 == curve ? pos2 : pos1;

		// Check if we need to swap ends
		{
			Vec2d origCoord = n1 == curve ? c1 : c2;

			Vec2d invCoord = inverseCoord(origCoord);
			Vector3 invPos(0);
			curve->get(coord3(invCoord), invPos);

			double dist_inv = (invPos - fixed).norm();

			// Swap coordinates for bad orientations
			if(dist_inv < dist)
				e.setCoord( curve->id, invCoord );
		}

		// Control point to move
		Vec2d curveCoord = e.getCoord(curve->id);
		int cpIdx = curve->controlPointIndexFromCoord( curveCoord );

		// Compute trajectory to closet on other node
		cpointIdx.push_back( cpIdx );
		curves.push_back( curve );
		deltas.push_back( fixed - curve->controlPoint( cpIdx ) );
	}

	// Generate target mesh
	//SurfaceMeshModel m;
	//g2->materialize(&m);
	//DynamicVoxel::MeanCurvatureFlow(&m, 0.05);
	//write_off(m, "sequence_.off");

	int NUM_STEPS = 30;
	int NUM_SMOOTH_ITR = 3;

	for(int s = 0; s < NUM_STEPS; s++)
	{
		for(int i = 0; i < (int)cpointIdx.size(); i++)
		{
			std::vector<Vector3> & cpnts = curves[i]->curve.mCtrlPoint;

			Vector3 & cp = cpnts[ cpointIdx[i] ];
			Vector3 delta = deltas[i] / NUM_STEPS;
			cp += delta;

			// Laplacian smoothing
			for(int itr = 0; itr < NUM_SMOOTH_ITR; itr++)
			{
				QVector<Vector3> newCtrlPnts;

				for (int j = 1; j < (int)cpnts.size() - 1; j++)
					newCtrlPnts.push_back( (cpnts[j-1] + cpnts[j+1]) / 2.0 );

				for (int j = 1; j < (int)cpnts.size() - 1; j++)
					cpnts[j] = newCtrlPnts[j-1];
			}
		}

		SurfaceMeshModel meshStep;
		g3.materialize(&meshStep);
		//DynamicVoxel::MeanCurvatureFlow(&meshStep, 0.05);
		DynamicVoxel::LaplacianSmoothing(&meshStep);
		DynamicVoxel::LaplacianSmoothing(&meshStep);

		// output step
		QString sequence_num;
		sequence_num.sprintf("%02d", s);

		QString fileName = QString("sequence_%1.off").arg(sequence_num);
		meshStep.triangulate();
		write_off(meshStep, qPrintable(fileName));
	}

	setSceneBounds();
}

Q_EXPORT_PLUGIN(topoblend)
