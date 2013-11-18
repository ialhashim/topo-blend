#include <QFileDialog>
#include "GraphModifyWidget.h"
#include "QuickAlignment.h"

#include "TopoBlender.h"
#include "GraphCorresponder.h"
#include "Scheduler.h"

#include "graphs-manager.h"
#include "SynthesisManager.h"

void GraphsManager::loadModel()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, "Open Model",
        tb->mainWindow()->settings()->getString("lastUsedDirectory"), "Model Files (*.xml)");
    if(fileNames.isEmpty()) return;

    // Keep folder active
    QFileInfo fileInfo(fileNames.front());
    tb->mainWindow()->settings()->set( "lastUsedDirectory", fileInfo.absolutePath() );

    foreach(QString file, fileNames)
    {
        tb->graphs.push_back( new Structure::Graph ( file ) );
    }

    tb->mainWindow()->setStatusBarMessage( "Loaded: \n" + fileNames.join("\n") );

    tb->setSceneBounds();
}

void GraphsManager::saveModel()
{
    if (tb->graphs.size() < 1){
        qDebug() << "Please load a graph.";
        return;
    }

    foreach(Structure::Graph * g, tb->graphs)
        g->setColorAll(Qt::lightGray);

    foreach(Structure::Graph * g, tb->graphs)
    {
        // Highlight selected graph
        g->setColorAll(Qt::yellow);
        tb->drawArea()->updateGL();

        QString filename = QFileDialog::getSaveFileName(0, "Save Model",
            g->property["name"].toString(), "Model Files (*.xml)");

        // Un-highlight
        g->setColorAll(Qt::lightGray);

        if(filename.isEmpty()) 	continue;

        g->saveToFile(filename);
    }
}

void GraphsManager::modifyModel()
{
    if (tb->graphs.size() < 1){
        qDebug() << "Please load a graph";
        return;
    }

    tb->widget->setCheckOption("showEdges");

    tb->viz_params["showNames"] = true;

    GraphModifyWidget modifyDialog(tb->graphs.back());
    tb->drawArea()->connect(&modifyDialog, SIGNAL(updateView()), SLOT(updateGL()));
    modifyDialog.exec();
}

void GraphsManager::quickAlign()
{
    if (tb->graphs.size() < 1){
        qDebug() << "Please load a graph";
        return;
    }

    QuickAlignment alignmentDialog(tb->graphs[0], tb->graphs[1]);
    tb->drawArea()->connect(&alignmentDialog, SIGNAL(updateView()), SLOT(updateGL()));
    alignmentDialog.exec();
}

void GraphsManager::normalizeAllGraphs()
{
    foreach (Structure::Graph* g, tb->graphs)
    {
        g->moveBottomCenterToOrigin();
        g->normalize();
    }

    tb->drawArea()->updateGL();
}

void GraphsManager::clearGraphs()
{
	// Clear any synthesis data
	tb->s_manager->clear();

    // UI and geometry clean up
    Scheduler * scheduler = tb->scheduler;
    if(scheduler)
    {
        scheduler->activeGraph->clearAll();
        scheduler->targetGraph->clearAll();

        // Remove old signals
        scheduler->disconnect(tb);
        tb->disconnect(scheduler);

        scheduler->dock->close();
    }

	// Delete corresponder, blender, and scheduler
	if(tb->gcoor) delete tb->gcoor;
	if(tb->blender) delete tb->blender;
	if(tb->scheduler) delete tb->scheduler;

	// Delete all graphs
	if(tb->graphs.size() > 1) qDeleteAll(tb->graphs);
	tb->graphs.clear();

	tb->gcoor = NULL;
	tb->blender = NULL;
	tb->scheduler = NULL;

    // Clear debug
    tb->debugPoints.clear(); tb->debugPoints2.clear();

    tb->updateDrawArea();
}
