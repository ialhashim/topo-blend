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
Q_DECLARE_METATYPE( Task* )

VectorSoup vs1, vs2;
double boundX = -DBL_MAX;

#include "graphs-manager.h"
#include "correspondence-manager.h"
#include "synthesis-manager.h"

topoblend::topoblend()
{
	widget = NULL;
    gcoor = NULL;
	blender = NULL;
	scheduler = NULL;

    orgSource = NULL;
    orgTarget = NULL;

    g_manager = new GraphsManager(this);
    c_manager = new CorrespondenceManager(this);
    s_manager = new SynthesisManager(this);
}

void topoblend::create()
{
	if(!widget)
	{
        ModePluginDockWidget * dockwidget = new ModePluginDockWidget("TopoBlender", mainWindow());
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
		graphs[g]->property["posX"] = posX;
		glPopMatrix();

		posX += deltaX;
	}

	if( scheduler && graphs.size() == 2 )
	{
		double posX = graphs.front()->property["posX"].toDouble();

		// Source
		foreach(Node * n, scheduler->activeGraph->nodes){
			glPushMatrix();
			glTranslatef(posX,0,0);
			if(n->vis_property["glow"].toBool()) n->draw();
			glPopMatrix();
		}
	}

	// Textual information
	glColor4d(1,1,1,0.25);
	drawArea()->renderText(40,40, "[TopoBlend]");
	if( property["correspondenceMode"].toBool() ) 
	{
		glColor4d(1,1,1,1);
		drawArea()->renderText(40,80, "Correspondence Mode");
	}
}

void topoblend::drawWithNames()
{
    float deltaX = drawArea()->sceneRadius();
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
	if(!graphs.size()) return;

	// Set scene bounds
	boundX = -DBL_MAX;

	Eigen::AlignedBox3d bigbox = graphs.front()->bbox();
	for(int i = 0; i < (int)graphs.size(); i++)
	{
		bigbox = bigbox.merged( Eigen::AlignedBox3d(graphs[i]->bbox()) );
		boundX = qMax(boundX, graphs[i]->bbox().diagonal().x());
	}

	// Scale up by 3
	//bigbox.transform(QMatrix4x4() * 3);

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
			Graph * sourceGraph = graphs.front();
			Graph * targetGraph = graphs.back();

			QVector<QString> sParts, tParts;

			// Source
			foreach(Node * n, sourceGraph->nodes){
				if( n->property["nodeSelected"].toBool() ){
					sParts << n->id;
					n->property["nodeSelected"] = false;
					n->vis_property["meshColor"].setValue( QColor(180,180,180) );
				}
			}

			// Target
			foreach(Node * n, targetGraph->nodes){
				if( n->property["nodeSelected"].toBool() ){
					tParts << n->id;
					n->property["nodeSelected"] = false;
					n->vis_property["meshColor"].setValue( QColor(180,180,180) );
				}
			}
			
			// Correspondence specified
			if(sParts.size() > 0 && tParts.size() > 0)
			{
				if(!gcoor) gcoor = makeCorresponder();

				gcoor->addLandmarks(sParts, tParts);

				QColor curColor = qRandomColor3(0, 0.25);

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

				gcoor->isReady = false;
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

		if(gcoor) gcoor->clear();

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

		// Color previously assigned correspondences
		if( gcoor ) {
			Graph * sourceGraph = graphs.front();
			Graph * targetGraph = graphs.back();

			foreach (PART_LANDMARK vector2vector, gcoor->correspondences){
				QColor curColor = qRandomColor3(0, 0.25);

				foreach (QString strID, vector2vector.first){
					sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
					sourceGraph->getNode( strID )->property["is_corresponded"] = true;
				}
			
				foreach (QString strID, vector2vector.second){
					targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
					targetGraph->getNode( strID )->property["is_corresponded"] = true;
				}
			}

			foreach (PART_LANDMARK vector2vector, gcoor->landmarks){
				QColor curColor = qRandomColor3(0, 0.25);

				foreach (QString strID, vector2vector.first){
					sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
					sourceGraph->getNode( strID )->property["is_corresponded"] = true;
				}
				
				foreach (QString strID, vector2vector.second){
					targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
					targetGraph->getNode( strID )->property["is_corresponded"] = true;
				}
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
					QVector<Eigen::Vector3f> pnts = node->property["cached_points"].value< QVector<Eigen::Vector3f> >();
					QVector<ParameterCoord> samples = node->property["samples"].value< QVector<ParameterCoord> >();
					QVector<float> offsets = node->property["offsets"].value< QVector<float> >();

					Vector3f delta = Vector3f(posX,0,0);
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
		else
		{
			QDir dir("");
			dir.setCurrent(QFileDialog::getExistingDirectory());

			DynamicGraphs::DynamicGraph sdg(graphs.front());
			toGraphviz(sdg, graphs.front()->name(), true, QString("V = %1, E = %2").arg(sdg.nodes.size()).arg(sdg.edges.size()), "Graph");
		}

		used = true;
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
	if ( graphs.size() < 2 && !orgSource )
	{
		qDebug() << "Please load at least two graphs.";
		return;
	}

	// Reload previous source and target if any
	if( graphs.size() < 2 )
	{
		// Use previously loaded 
		this->graphs.clear();
		this->graphs.push_back( new Graph(*orgSource) );
		this->graphs.push_back( new Graph(*orgTarget) );
	}

	orgSource = graphs.front();
	orgTarget = graphs.back();

	// Visualization
	foreach(Graph * g, graphs){
		foreach(Node * n, g->nodes){
			n->vis_property["meshSolid"] = false;
			n->vis_property["meshColor"].setValue( QColor(200,200,200,8) );
		}
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
		gcoor = makeCorresponder();
	}

	if( !gcoor->isReady )
	{
		gcoor->clear();
		gcoor->computeCorrespondences();
	}

	scheduler = new Scheduler( );
    blender = new TopoBlender( gcoor, scheduler );

	// Update active graph
	this->connect(scheduler, SIGNAL(activeGraphChanged( Structure::Graph* )), SLOT(updateActiveGraph( Structure::Graph* )));

	// Render connections
	this->connect(scheduler, SIGNAL(renderAll()), SLOT(renderAll()), Qt::UniqueConnection);
	this->connect(scheduler, SIGNAL(renderCurrent()), SLOT(renderCurrent()), Qt::UniqueConnection);
	this->connect(scheduler, SIGNAL(draftRender()), SLOT(draftRender()), Qt::UniqueConnection);

	// Other connections
	this->connect(scheduler, SIGNAL(updateExternalViewer()), SLOT(updateDrawArea()));

	setStatusBarMessage( QString("Created TopoBlender and tasks in [ %1 ms ]").arg( timer.elapsed() ) );

	//this->graphs.clear();
	//this->graphs.push_back(scheduler->activeGraph);
	//this->graphs.push_back(scheduler->targetGraph);

	setSceneBounds();

	updateDrawArea();
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

	if(!scheduler->property["synthDataReady"].toBool())
		this->widget->setCheckOption("showMeshes", false);

	drawArea()->updateGL();
}

void topoblend::setStatusBarMessage(QString message)
{
	mainWindow()->setStatusBarMessage(message, 1000);
	drawArea()->updateGL();
}

Q_EXPORT_PLUGIN(topoblend)
