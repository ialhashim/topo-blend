#include <QElapsedTimer>
#include <QFileDialog>
#include <QDialog>
#include <QStack>
#include <QQueue>

#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "topo-blend.h"

#include "../CustomDrawObjects.h"
#include "graph_modify_dialog.h"
#include "QuickAlignment.h"
#include "GraphExplorer.h"

using namespace NURBS;
using namespace Structure;

#include "DynamicGraph.h"
#include "ExportDynamicGraph.h"
#include "GraphDistance.h"
#include "GraphCorresponder.h"
#include "TopoBlender.h"
#include "Scheduler.h"
#include "Task.h"
#include "Synthesizer.h"

#include "graphs-manager.h"
#include "correspondence-manager.h"
#include "SynthesisManager.h"

#define BBOX_WIDTH(box) (box.max().x()-box.min().x())
#define PADDING_FACTOR 1.0

// Simple UI
#include "wizard.h"

#include "QuickMeshDraw.h"

topoblend::topoblend()
{
	widget = NULL;
	wizard = NULL;
	graph_explorer = NULL;

    gcoor = NULL;
	blender = NULL;
	scheduler = NULL;

    g_manager = new GraphsManager(this);
    c_manager = new CorrespondenceManager(this);
    s_manager = new SynthesisManager(NULL, NULL, NULL);
}

void topoblend::create()
{
	if(!widget)
	{
        ModePluginDockWidget * dockwidget = new ModePluginDockWidget("TopoBlender", mainWindow());
		widget = new topo_blend_widget(this);
		dockwidget->setWidget(widget);
		dockwidget->setWindowTitle(widget->windowTitle());
		mainWindow()->addDockWidget(Qt::RightDockWidgetArea, dockwidget);

		points = mesh()->vertex_property<Vector3>("v:point");

		// Events
		this->connect(this, SIGNAL(statusBarMessage(QString)), SLOT(setStatusBarMessage(QString)));
		this->connect(s_manager, SIGNAL(setMessage(QString)), SLOT(setStatusBarMessage(QString)));
		this->connect(s_manager, SIGNAL(updateViewer()), SLOT(updateDrawArea()));

		// Simple UI
		this->wizard = new Wizard(this, widget->simpleWidget());

		// Explore graph and its properties
		this->graph_explorer = new GraphExplorer;
	}

	drawArea()->setSelectRegionHeight( 20 );
	drawArea()->setSelectRegionWidth( 20 );

	drawArea()->setShortcut(QGLViewer::DRAW_AXIS, Qt::Key_A);
	drawArea()->setShortcut(QGLViewer::DRAW_GRID, Qt::Key_G);

	// Change camera type
	drawArea()->camera()->setType(qglviewer::Camera::PERSPECTIVE);
	setSceneBounds();
}

void topoblend::drawFancyBackground()
{
	// Fancy background
	int width = drawArea()->width();
	int height = drawArea()->height();
	drawArea()->startScreenCoordinatesSystem();
	glDisable(GL_LIGHTING);
	glBegin(GL_QUADS);
	glColor3d(0,0,0); glVertex2d(0,0); glVertex2d(width,0);
	glColor3d(0.2,0.2,0.2); glVertex2d(width,height); glVertex2d(0,height);
	glEnd();
	glEnable(GL_LIGHTING);
	glClear(GL_DEPTH_BUFFER_BIT);
	drawArea()->stopScreenCoordinatesSystem();
}

void topoblend::decorate()
{
	drawFancyBackground();

	// Make sure we draw smooth objects
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//drawBBox(bigbox);

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

	glPopMatrix();

	// Points 2
	glPushMatrix();
	glTranslatef(0.0, drawArea()->sceneRadius(), 0);
	glColor3d(0,1,0); glBegin(GL_POINTS); foreach(Vector3 v, debugPoints2) glVector3(v); glEnd();
	
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

	if( scheduler && scheduler->allGraphs.size() == 0 )
	{
		QVector<Structure::Graph*> currentGraphs;
		currentGraphs.push_back(scheduler->activeGraph);
		currentGraphs.push_back(scheduler->targetGraph);

		// Visualize tasks
		glDisable(GL_LIGHTING);
		foreach(Structure::Graph * g, currentGraphs){
			foreach(Node * n, g->nodes){
				if(!n->vis_property["glow"].toBool() || n->id.contains("_null")) continue;

				int nType = n->property["taskTypeReal"].toInt();

				double posX = g->property["posX"].toDouble();

				QString taskName = TaskNames[nType];

				glColorQt(TaskColors[nType]);
				
				qglviewer::Vec end(drawArea()->width() * 0.5, 20, 0);

				// Draw arc
				drawArea()->startScreenCoordinatesSystem();

				Vector3 boxCenter = n->bbox().center();
				boxCenter.x() += posX;

				qglviewer::Vec start = drawArea()->camera()->projectedCoordinatesOf( qglviewer::Vec(boxCenter) );

				double bend = currentGraphs.front() == g ? -1 : 1;
				std::vector<Vector3> arc = BezierArc(Vector3(start.x,start.y,start.z), Vector3(end.x,end.y,end.z), 10, bend);

				for(int i = 0; i < (int) arc.size(); i++){
					glLineWidth(3);
					glBegin(GL_LINE_STRIP);
					foreach(Vector3 p, arc) glVector3(p);
					glEnd();
				}

				// Draw box
				int w = 30, h = 10;
				glBegin (GL_POLYGON);
				glVertex2d (end.x - w, end.y - h);
				glVertex2d (end.x + w, end.y - h);
				glVertex2d (end.x + w, end.y + h);
				glVertex2d (end.x - w, end.y + h);
				glEnd();

				// Draw text
				QFont font = QFont();
				QFontMetrics metric(font);
				glColor3d(0,0,0);
				drawArea()->renderText(end.x - (metric.width(taskName) * 0.5), end.y + 4, taskName);

				drawArea()->stopScreenCoordinatesSystem();

				glEnable(GL_MULTISAMPLE);
			}
		}
		glEnable(GL_LIGHTING);

		// Draw nodes both selected and regular
		foreach(Structure::Graph * g, currentGraphs){
			foreach(Node * n, g->nodes){
				if(n->id.contains("_null")) continue;

				double posX = g->property["posX"].toDouble();

				glPushMatrix();
				glTranslated(posX, 0, 0);

				SurfaceMesh::Model* nodeMesh = n->property["mesh"].value<SurfaceMesh::Model*>();

				if( !nodeMesh || !nodeMesh->n_vertices() || !viz_params["showMeshes"].toBool() )
				{
					if(viz_params["showNodes"].toBool())
						n->draw();
				}
				else
				{
					if(n->vis_property["glow"].toBool()) 
					{
						QColor meshColor(255,255,0);
						QuickMeshDraw::drawMeshSolid( nodeMesh, meshColor );
					}
					else
					{
						QColor meshColor(180,180,180,220);
						QuickMeshDraw::drawMeshSolid( nodeMesh, meshColor );
					}
				}
	
				glPopMatrix();
			}
		}

		// Draw synthesis samples if available
		if(viz_params["showSamples"].toBool())
			s_manager->drawSampled();
	}
	else if(graphs.size())
	{
		double startX = bigbox.min().x();

		for(int g = 0; g < (int) graphs.size(); g++)
		{
			// Apply visualization options
			graphs[g]->property["showEdges"] = viz_params["showEdges"];
			graphs[g]->property["showMeshes"] = viz_params["showMeshes"];
			graphs[g]->property["showTasks"] = viz_params["showTasks"];
			graphs[g]->property["showCtrlPts"] = viz_params["showCtrlPts"];
			graphs[g]->property["showNodes"] = viz_params["showNodes"];
			graphs[g]->property["showNames"] = viz_params["showNames"];
			graphs[g]->property["showCurveFrames"] = viz_params["showCurveFrames"];

			// Place and draw graph
			glPushMatrix();

			Eigen::AlignedBox3d curbox = graphs[g]->bbox();

			double curwidth = (curbox.max().x() - curbox.min().x());
			double deltaX = curwidth * 0.5;

			double padding = 0;
			if(g > 0) padding = curwidth * PADDING_FACTOR;

			double posX = startX + deltaX + padding;

			if(graphs.size() < 2) posX = 0;

			glTranslated(posX, 0, 0);

			// store for later use
			graphs[g]->property["posX"] = posX;
			graphs[g]->draw( drawArea() );
			//drawBBox( curbox );

			glPopMatrix();

			startX += curwidth + padding;
		}

		if( scheduler && scheduler->allGraphs.size() )
		{
			s_manager->isSplatRenderer = viz_params["isSplatRenderer"].toBool();
			s_manager->splatSize = viz_params["splatSize"].toDouble();

			s_manager->drawSynthesis( graphs.back() );
		}
	}

	// Textual information
	glColor4d(1,1,1,0.25);
	drawArea()->renderText(40,40, "[TopoBlend]");
	if( property["correspondenceMode"].toBool() ) 
	{
		glColor4d(1,1,1,1);
		drawArea()->renderText(40,60, "Correspondence Mode");
	}
}

void topoblend::drawWithNames()
{
	if(property["correspondenceMode"].toBool())
	{
		c_manager->drawWithNames();
		return;
	}

	// Select control points
	for(int gID = 0; gID < (int) graphs.size(); gID++)
	{
		Structure::Graph *g = graphs[gID];
		int nodeID_base = gID * NODE_ID_RANGE;

		glPushMatrix();
		glTranslated(g->property["posX"].toDouble(), 0, 0);

		for (int nID = 0; nID < (int)g->nodes.size(); nID++)
		{
			g->nodes[nID]->drawWithNames(nodeID_base + nID, POINT_ID_RANGE);
		}

		glPopMatrix();
	}
}

bool topoblend::postSelection( const QPoint& point )
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

        return false;
	}

    if (selectedID == -1) return false;

	int gID, nID, pID;
	getIndicesFromSelectedName(selectedID, gID, nID, pID);

	graphs[gID]->nodes[nID]->addSelectionWithColor(pID, Qt::green);

	qDebug() << "Selected ID is " << selectedID;

    return true;
}

void topoblend::setSceneBounds()
{
	if(!graphs.size())
	{
		drawArea()->setSceneRadius(2);
		drawArea()->setSceneCenter(qglviewer::Vec(0,0,0));
		drawArea()->setSceneBoundingBox(qglviewer::Vec(-1,-1,-1), qglviewer::Vec(1,1,1));
		drawArea()->camera()->setPosition(qglviewer::Vec(-1,-3,2));
		drawArea()->showEntireScene();
		drawArea()->updateGL();
		return;
	}

	// Set scene bounds
	bigbox = graphs.front()->bbox();
	double deltaX = BBOX_WIDTH(bigbox);
	bigbox.translate( Vector3(deltaX * 0.5, 0, 0) ); // start from zero

	for(int i = 1; i < (int)graphs.size(); i++)
	{
		Eigen::AlignedBox3d curbox = graphs[i]->bbox();
		
		double curWidth = BBOX_WIDTH(curbox);
		double padding = curWidth * PADDING_FACTOR;

		curbox.translate( Vector3(deltaX + (0.5 * curWidth) + padding, 0, 0) );
		bigbox = bigbox.merged( Eigen::AlignedBox3d(curbox) );

		deltaX += BBOX_WIDTH(curbox) + padding; 
	}

	// Move to center
	bigbox.translate( Vector3(-bigbox.center().x(), 0, 0) );

	Vector3 a = bigbox.min();
	Vector3 b = bigbox.max();

	qglviewer::Vec vecA(a.x(), a.y(), a.z());
	qglviewer::Vec vecB(b.x(), b.y(), b.z());

	drawArea()->setSceneCenter((vecA + vecB) * 0.5);
	drawArea()->setSceneBoundingBox(vecA, vecB);
	drawArea()->showEntireScene();
	drawArea()->updateGL();
}

bool topoblend::mouseReleaseEvent( QMouseEvent * event )
{
	bool used = false;

	if( property["correspondenceMode"].toBool() ) 
	{
		if( event->button() == Qt::RightButton )
		{
			c_manager->assignCorrespondence();
		}
	}

	return used;
}

bool topoblend::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	QElapsedTimer timer; timer.start();

	if(event->key() == Qt::Key_I)
	{
		if(graphs.size()){
			graph_explorer->update(graphs.front());
			if(graph_explorer) graph_explorer->show();
		}

		used = true;
	}

	if(event->key() == Qt::Key_C)
	{
		c_manager->clearCorrespondence();
		used = true;
	}

	if(event->key() == Qt::Key_Space)
	{
		c_manager->correspondenceMode();
		used = true;
	}

    /*
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
    */

	if(event->key() == Qt::Key_W)
	{
		if(graphs.size() < 1) return true;

		Structure::Graph * g = graphs.back();

		QElapsedTimer t; t.start();

		GraphDistance * gd = new GraphDistance(g);

		std::vector<Vector3> starts;
		starts.push_back(Vector3(1,-0.5,2.5));
		starts.push_back(Vector3(-1,-0.5,2.5));

		gd->computeDistances(starts, g->bbox().diagonal().norm() * 0.01);
		g->misc["distance"] = gd;

		mainWindow()->setStatusBarMessage( "Distance test: " + QString::number(t.elapsed()) );

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
		Vector3d p(q[0],q[1],q[2]);

		double minDist = DBL_MAX;
		QString sample_details = "";

		for(int gi = 0; gi < (int) graphs.size(); gi++)
		{
			Structure::Graph * g = graphs[gi];
			g->property.remove("selectedSample");

			foreach(Structure::Node * node, g->nodes)
			{
				if(node->property.contains("cached_points"))
				{
					/*QVector<Eigen::Vector3f> & pnts = *(node->property["cached_points"].value< QVector<Eigen::Vector3f>* >());
					QVector<ParameterCoord> & samples = *(node->property["samples"].value< QVector<ParameterCoord>* >());
					QVector<float> & offsets = *(node->property["offsets"].value< QVector<float>* >());

					Vector3f delta = Vector3f(g->property["posX"].toDouble(),0,0);
					Vector3f q = p.cast<float>() - delta;

					for(int i = 0; i < (int)pnts.size(); i++)
					{
						double dist = Vector3f(pnts[i] - q).norm();
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
					}*/
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

			used = true;
		}
		else if(graphs.size())
		{
			QDir dir("");
			dir.setCurrent(QFileDialog::getExistingDirectory());

			DynamicGraphs::DynamicGraph sdg(graphs.front());
			toGraphviz(sdg, graphs.front()->name(), true, QString("V = %1, E = %2").arg(sdg.nodes.size()).arg(sdg.edges.size()), "Graph");
			
			used = true;
		}
	}

	if(event->key() == Qt::Key_F)
	{
		QString graphFilename = "tempGraph_"+QString::number(QDateTime::currentMSecsSinceEpoch());
		visualizeStructureGraph(graphs.back(), graphFilename, "Current graph");

		QImage img(graphFilename+".png");

		QMessageBox* msgBox = new QMessageBox;
		msgBox->setAttribute( Qt::WA_DeleteOnClose );
		msgBox->setIconPixmap( QPixmap::fromImage(img) );
		msgBox->setModal( false );
		msgBox->setMinimumSize(img.width(), img.height());
		msgBox->open();

		used = true;
	}

	drawArea()->updateGL();

	return used;
}

void topoblend::doBlend()
{
	if ( graphs.size() < 2 )
	{
		qDebug() << "Please load at least two graphs.";
		return;
	}

	QElapsedTimer timer; timer.start();

	if(scheduler)
	{
		// Old signals
		scheduler->disconnect(this);
		this->disconnect(scheduler);
		scheduler->dock->close();
		scheduler = NULL;
	}

	if( !gcoor )
	{
		gcoor = c_manager->makeCorresponder();
	}

	if( !gcoor->isReady )
	{
		gcoor->computeCorrespondences();
	}

	c_manager->exitCorrespondenceMode(true);

	scheduler = new Scheduler( );
    blender = new TopoBlender( gcoor, scheduler );

	blender->setupUI();

	// Update active graph
	this->connect(scheduler, SIGNAL(activeGraphChanged( Structure::Graph* )), SLOT(updateActiveGraph( Structure::Graph* )));

	// Render connections
	this->s_manager->connect(scheduler, SIGNAL(renderAll()), SLOT(renderAll()), Qt::UniqueConnection);
	this->s_manager->connect(scheduler, SIGNAL(renderCurrent()), SLOT(renderCurrent()), Qt::UniqueConnection);

	// Other connections
	this->connect(scheduler, SIGNAL(updateExternalViewer()), SLOT(updateDrawArea()));

	setSceneBounds();
	updateDrawArea();

	setStatusBarMessage( QString("Created TopoBlender and tasks in [ %1 ms ]").arg( timer.elapsed() ) );
}

void topoblend::updateDrawArea()
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
	if(s_manager->renderData.size())
		widget->setCheckOption("showNodes", false);
	widget->setCheckOption("showMeshes", false);

	graphs.clear();

	this->graphs.push_back(newActiveGraph);

	drawArea()->updateGL();

	if(graph_explorer->isVisible()){
		graph_explorer->update(newActiveGraph);
	}
}

void topoblend::setStatusBarMessage(QString message)
{
	mainWindow()->setStatusBarMessage(message, 1000);
	drawArea()->updateGL();
}

void topoblend::drawBBox(Eigen::AlignedBox3d bbox)
{
	float min[3]; 
	min[0] = bbox.min().x();
	min[1] = bbox.min().y();
	min[2] = bbox.min().z();

	float max[3]; 
	max[0] = bbox.max().x();
	max[1] = bbox.max().y();
	max[2] = bbox.max().z();

	/// --- Inherited from VCG ---
	glPushAttrib(GL_ENABLE_BIT);
	glColor3d(1,1,0.5);
	glLineWidth(3);
	glLineStipple(3, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);

	glDisable(GL_LIGHTING);
	glBegin(GL_LINE_STRIP);
	glVertex3f((float)min[0],(float)min[1],(float)min[2]);
	glVertex3f((float)max[0],(float)min[1],(float)min[2]);
	glVertex3f((float)max[0],(float)max[1],(float)min[2]);
	glVertex3f((float)min[0],(float)max[1],(float)min[2]);
	glVertex3f((float)min[0],(float)min[1],(float)min[2]);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3f((float)min[0],(float)min[1],(float)max[2]);
	glVertex3f((float)max[0],(float)min[1],(float)max[2]);
	glVertex3f((float)max[0],(float)max[1],(float)max[2]);
	glVertex3f((float)min[0],(float)max[1],(float)max[2]);
	glVertex3f((float)min[0],(float)min[1],(float)max[2]);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f((float)min[0],(float)min[1],(float)min[2]);
	glVertex3f((float)min[0],(float)min[1],(float)max[2]);
	glVertex3f((float)max[0],(float)min[1],(float)min[2]);
	glVertex3f((float)max[0],(float)min[1],(float)max[2]);
	glVertex3f((float)max[0],(float)max[1],(float)min[2]);
	glVertex3f((float)max[0],(float)max[1],(float)max[2]);
	glVertex3f((float)min[0],(float)max[1],(float)min[2]);
	glVertex3f((float)min[0],(float)max[1],(float)max[2]);
	glEnd();
	glPopAttrib(); 
}

bool topoblend::mousePressEvent( QMouseEvent * event )
{
	property["mousePressPos"] = event->pos();
	return false;
}

bool topoblend::mouseMoveEvent( QMouseEvent * event )
{
	if( property["correspondenceMode"].toBool() ) 
	{
		if( event->buttons() & Qt::LeftButton ){
			QPoint startPos = property["mousePressPos"].toPoint();
			if((event->pos()-startPos).manhattanLength()){
				setStatusBarMessage(QString::number(event->pos().x()));

				qglviewer::Vec orig, dir;
				drawArea()->camera()->convertClickToLine(event->pos(), orig, dir);

				for(int gID = 0; gID < (int) graphs.size(); gID++)
				{
					Structure::Graph *g = graphs[gID];

					foreach(Node * n, g->nodes){
						SurfaceMesh::Model* nodeMesh = n->property["mesh"].value<SurfaceMesh::Model*>();
						double posX = g->property["posX"].toDouble();

						
					}
				}
			}
		}
	}
	return false;
}

bool topoblend::rayBBoxIntersect( Eigen::AlignedBox3d bbox, Vector3 origin, Vector3 ray )
{
	return false;
}

Q_EXPORT_PLUGIN(topoblend)
