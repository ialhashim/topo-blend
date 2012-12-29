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


#define RESOLUTION 0.1


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
	
	Morpher * morpher = new Morpher(sourceModel,targetModel,sourceCurve,targetCurve);
//	Morpher *morpher=new Morpher(sourceModel,targetModel,sourceGraph,targetGraph);
	
	morpher->generateInBetween(10,15,20);

}


Q_EXPORT_PLUGIN(geometry_morph)
