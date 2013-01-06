#include <QElapsedTimer>
#include <QFileDialog>
#include <QStack>
#include <QQueue>

#include "geometry_morph.h"
#include "StarlabMainWindow.h"
#include "StarlabDrawArea.h"
#include "interfaces/ModePluginDockWidget.h"
#include "../CustomDrawObjects.h"

//#include "StructureGraph.h"
//#include "DynamicGraph.h"

#include "Morpher.h"
Morpher *morpher = NULL;

#define RESOLUTION 0.1

// Test
#include "Octree.h"
Octree * octree = NULL;
Ray ray(Vec3d(0),Vec3d(1,1,1));

geometry_morph::geometry_morph(){
	widget = NULL;
//	sourceModel=NULL;
//	targetModel=NULL;
//	sourceCurve = NURBSCurve::NURBSCurve();
//	targetCurve=NURBSCurve::NURBSCurve();
}

void geometry_morph::create()
{
	if(!widget)
	{
		ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
		widget = new geometry_morph_widget(this);
		dockwidget->setWidget(widget);
		dockwidget->setWindowTitle(widget->windowTitle());
		mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);
	}
}

void geometry_morph::decorate()
{
	//targetGraph.draw();

	if(octree)
	{
		HitResult res;
		octree->intersectRay(ray, ray.thickness, false);

		octree->draw(0,1,0,1);


		// Draw ray

		glDisable(GL_LIGHTING);

		glLineWidth(10);
		glColor3d(1,1,1);
		glBegin(GL_LINES);
		glVector3(ray.origin);
		glVector3((ray.origin + ray.direction * 100));
		glEnd();

		glEnable(GL_LIGHTING);
	}

	if(morpher)
	{
		if(morpher->source_octree) morpher->source_octree->draw(0,1,0,1);
		
		glDisable(GL_LIGHTING);
		glPointSize(15);
		glBegin(GL_POINTS);

		glColor3d(1,0,0);
		foreach(Vec3d p, morpher->debugPoints)	glVector3(p);

		glColor3d(0,1,0);
		foreach(Vec3d p, morpher->debugPoints2)	glVector3(p);

		glEnd();
		glEnable(GL_LIGHTING);
	}
}

/*
void geometry_morph::generateTwoNurbsCurves()
{
	
	QElapsedTimer timer; timer.start();

	Structure::Graph source, target;
	
	sourceCurve = NURBSCurve::createCurve(Vector3(-1,1.75,0), Vector3(-1,1.9,-2));

//	sourceCurve.SetTimeInterval(0,1);
	targetCurve = NURBSCurve::createCurve(2*Vector3(-1,1.75,0), 2*Vector3(-1,1.9,-2));
	//targetCurve.SetTimeInterval(0,1);

	source.addNode(new Structure::Curve(sourceCurve, "source_curve") );
	target.addNode(new Structure::Curve(targetCurve,"target_curve"));

	//source.saveToFile("source_curve1.xml");
	//target.saveToFile("target_curve1.xml");

	setSceneBounds();
}
*/

void geometry_morph::setSceneBounds()
{

}

void geometry_morph::loadSourceModel()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

	sourceModel = new SurfaceMeshModel(fileName,"sourceModel");
	sourceModel->read(fileName.toStdString());
	//setSceneBounds();

//	document()->addModel(sourceModel);
//	sourceModel->updateBoundingBox();

//	drawArea()->resetViewport();
}

void geometry_morph::loadTargetModel()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

	targetModel = new SurfaceMeshModel(fileName,"targetModel");
	targetModel->read(fileName.toStdString());
	//setSceneBounds();

//	document()->addModel(targetModel);
//	targetModel->updateBoundingBox();

//	drawArea()->resetViewport();
}

void geometry_morph::loadSourceCurve()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

	SurfaceMeshModel *sourceCurveModel = new SurfaceMeshModel(fileName,"sourceCurveModel");
	sourceCurveModel->read(fileName.toStdString());

	Vector3VertexProperty points = sourceCurveModel->vertex_property<Vector3>(VPOINT);
	std::vector<Vector3> ctrlPoints;

	foreach(Vertex v, sourceCurveModel->vertices())
		ctrlPoints.push_back(points[v]);

	std::vector<Real> ctrlWeight(ctrlPoints.size(), 1.0);

	sourceCurve=NURBSCurve(ctrlPoints, ctrlWeight, 3, false, true);
}

void geometry_morph::loadTargetCurve()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

	SurfaceMeshModel *targetCurveModel = new SurfaceMeshModel(fileName,"targetCurveModel");
	targetCurveModel->read(fileName.toStdString());

	Vector3VertexProperty points = targetCurveModel->vertex_property<Vector3>(VPOINT);
	std::vector<Vector3> ctrlPoints;

	foreach(Vertex v, targetCurveModel->vertices())
		ctrlPoints.push_back(points[v]);

	std::vector<Real> ctrlWeight(ctrlPoints.size(), 1.0);

	targetCurve=NURBSCurve(ctrlPoints, ctrlWeight, 3, false, true);
}

void geometry_morph::loadSourceGraph()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

	sourceGraph=Structure::Graph ( fileName) ;
}

void geometry_morph::loadTargetGraph()
{
	QString fileName = QFileDialog::getOpenFileName(0, tr("Open Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

	targetGraph=Structure::Graph ( fileName);
}

void geometry_morph::doMorph()
{

	//DEBUG:
	sourceModel = new SurfaceMeshModel("source.off","source");
	targetModel = new SurfaceMeshModel("target.off","target");
	sourceModel->updateBoundingBox();
	targetModel->updateBoundingBox();

	// Curve
	//sourceModel->read("C:/Users/rui/Desktop/StarlabPackage/cuboid7.off");
	//targetModel->read("C:/Users/rui/Desktop/StarlabPackage/cylinder13.off");
	//sourceGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_cuboid7.xml");
	//targetGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_cylinder13.xml");

	//// Sheet
    sourceModel->read("sheet1.off");
    targetModel->read("sheet2.off");
    sourceGraph.loadFromFile("graph_sheet1.xml");
    targetGraph.loadFromFile("graph_sheet2.xml");

	QElapsedTimer timer; timer.start();

	qDebug() << "Started 'generateInBetween'..";
	morpher = new Morpher(sourceModel,targetModel,sourceGraph,targetGraph);
	double t1=timer.elapsed();

    morpher->generateInBetween(3,60,60,0,80,80);
	double t2=timer.elapsed();

	qDebug() << QString("Morpher Constructor =%1 ms Resampling = %2 ms").arg(t1).arg(t2-t1);
	qDebug() << QString("Total =%1 ms").arg(t2);

	mainWindow()->setStatusBarMessage(QString("Total time = %1").arg(t2));

	//document()->addModel(sourceModel);
}

bool geometry_morph::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	if(event->key() == Qt::Key_E)
	{	
		std::vector<Surface_mesh::Face> allTris;
		foreach(Face f, mesh()->faces()) allTris.push_back(f);

		int numTrisPerNode =	15;

		octree = new Octree(numTrisPerNode, mesh());
		octree->initBuild(allTris, numTrisPerNode);

		used = true;
	}

	if(event->key() == Qt::Key_R)
	{	
		ray.thickness += 0.5;
		used = true;
		drawArea()->updateGL();
	}

	if(event->key() == Qt::Key_W)
	{	
		ray.thickness -= 0.5;
		used = true;
		drawArea()->updateGL();
	}

	return used;
}

Q_EXPORT_PLUGIN(geometry_morph)
