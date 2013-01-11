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
    QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"),
        mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

    foreach(QString file, fileNames)
    {
        sourceModels.push_back(new SurfaceMeshModel(file, "sourceModel") );
        sourceModels.last()->read(file.toStdString());
    }

//	document()->addModel(sourceModel);
//	sourceModel->updateBoundingBox();

//	drawArea()->resetViewport();
}

void geometry_morph::loadTargetModel()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"),
        mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.off *.obj)"));

    foreach(QString file, fileNames)
    {
        targetModels.push_back(new SurfaceMeshModel(file, "targetModel") );
        targetModels.last()->read(file.toStdString());
    }

//	document()->addModel(targetModel);
//	targetModel->updateBoundingBox();

//	drawArea()->resetViewport();
}

void geometry_morph::loadSourceGraph()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"),
        mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

    foreach(QString file, fileNames)
        sourceGraphs.push_back(Structure::Graph ( file ) );
}

void geometry_morph::loadTargetGraph()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, tr("Open Model"),
        mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

    foreach(QString file, fileNames)
        targetGraphs.push_back(Structure::Graph ( file ) );
}

void geometry_morph::doMorph()
{
    int numStep=10;
    int uResolution=20;
    int vResolution=20;
    int timeResolution=20;
    int thetaResolution=20;
    int phiResolution=12;

    //DEBUG:
    //sourceModel = new SurfaceMeshModel("source.off","source");
    //targetModel = new SurfaceMeshModel("target.off","target");
    //sourceModel->updateBoundingBox();
    //targetModel->updateBoundingBox();

    // Curve
    //sourceModel->read("C:/Users/rui/Desktop/StarlabPackage/cuboid7.off");
    //targetModel->read("C:/Users/rui/Desktop/StarlabPackage/cylinder13.off");
    //sourceGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_cuboid7.xml");
    //targetGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_cylinder13.xml");

    //// Sheet
    //sourceModel->read("C:/Users/rui/Desktop/StarlabPackage/sheet1.off");
    //targetModel->read("C:/Users/rui/Desktop/StarlabPackage/sheet2.off");
    //sourceGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_sheet1.xml");
    //targetGraph.loadFromFile("C:/Users/rui/Desktop/StarlabPackage/graph_sheet2.xml");

    QElapsedTimer timer; timer.start();

    qDebug() << "Started 'generateInBetween'..";

    for (int i=0; i<sourceModels.size(); i++)
    {
		// resampling happens in the constructor and results are saved in resampledSourceMesh
        morpher = new Morpher(sourceModels[i],targetModels[i],sourceGraphs[i],targetGraphs[i],
			uResolution,vResolution,timeResolution,thetaResolution, phiResolution);    
        //morpher->testCase();

        resampledSourceModels.push_back(morpher->resampledSourceMesh);
        resampledTargetModels.push_back(morpher->resampledTargetMesh);
    }

    for (int step=0; step<=numStep; step++)
    {
        double t=step*1.0/numStep;
		std::vector<double > T(resampledSourceModels.size(), t);
        blendedModels.push_back(morpher->generateInBetween(resampledSourceModels,resampledTargetModels,T));
        //blendedModels.last()->triangulate();

        QString seq_num; seq_num.sprintf("%02d", step);
        blendedModels.last()->write(QString("blend%1.off").arg(seq_num).toStdString());
    }

	mainWindow()->setStatusBarMessage(QString("Geometry morphing done: %1 ms").arg(timer.elapsed()));
    ////document()->addModel(sourceModel);
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
