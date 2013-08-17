#include <QFileDialog>
#include <QtConcurrentRun>
#include "synthesis-manager.h"

// Synthesis
#include "Synthesizer.h"
#include "normal_extrapolation.h"
#include "SimilarSampling.h"

// Reconstruction
#include "poissonrecon.h"

#include "GraphCorresponder.h"
#include "Scheduler.h"
#include "Task.h"

void SynthesisManager::genSynData()
{
    qApp->setOverrideCursor(Qt::WaitCursor);

    QElapsedTimer timer; timer.start();

    int numNodes = tb->scheduler->activeGraph->nodes.size();
    int n = 0;

    // Force generate
    foreach(Structure::Node * node, tb->scheduler->activeGraph->nodes)
    {
        if(node->property.contains("correspond"))
        {
            QString nodeID = node->id;
            QString tnodeID = node->property["correspond"].toString();
            Structure::Node * tgNode = tb->scheduler->targetGraph->getNode(tnodeID);
            Synthesizer::clearSynthData(node);
            Synthesizer::clearSynthData(tgNode);
        }
    }

    // Number of samples
    randomCount = tb->widget->synthesisSamplesCount();
    uniformTriCount = tb->widget->synthesisSamplesCount();

    // Generate synthesis data for each corresponding node
    foreach(Structure::Node * snode, tb->scheduler->activeGraph->nodes)
    {
        Structure::Node * tnode = tb->scheduler->targetGraph->getNode( snode->property["correspond"].toString() );

        //int sampling_method = Synthesizer::Random | Synthesizer::Features;
        int sampling_method = Synthesizer::TriUniform | Synthesizer::Features;
        //int sampling_method = Synthesizer::Features;
        //int sampling_method = Synthesizer::Uniform;
        //int sampling_method = Synthesizer::Remeshing;
        //int sampling_method = Synthesizer::Random | Synthesizer::Features | Synthesizer::TriUniform;

        if(snode->type() == Structure::CURVE)
        {
            Synthesizer::prepareSynthesizeCurve((Structure::Curve*)snode, (Structure::Curve*)tnode, sampling_method);
        }

        if(snode->type() == Structure::SHEET)
        {
            Synthesizer::prepareSynthesizeSheet((Structure::Sheet*)snode, (Structure::Sheet*)tnode, sampling_method);
        }

        int percent = (double(n) / (numNodes-1) * 100);
        emit( tb->statusBarMessage(QString("Generating data.. [ %1 % ]").arg(percent)) );
        n++;
    }

    // Copy to current displayed graphs
    //foreach(Node* n, scheduler->activeGraph->nodes) Synthesizer::copySynthData( n, graphs.front()->getNode(n->id.split("_").at(0)) );
    //foreach(Node* n, scheduler->targetGraph->nodes) Synthesizer::copySynthData( n, graphs.back()->getNode(n->id.split("_").at(0)) );

    QString timingString = QString("Synthesis data [ %1 ms ]").arg(timer.elapsed());
    qDebug() << timingString;
    emit( tb->statusBarMessage(timingString) );

    tb->scheduler->property["synthDataReady"] = true;

    qApp->restoreOverrideCursor();
}

void SynthesisManager::generateSynthesisData()
{
    if(!tb->blender) return;

    tb->setStatusBarMessage("Generating synthesis samples...");

#ifdef Q_OS_WIN
    QtConcurrent::run( this, &SynthesisManager::genSynData );
#else // OpenMP issue on OSX (Linux too?)
    genSynData();
#endif
}

void SynthesisManager::saveSynthesisData(QString parentFolder)
{
    if(!tb->blender) return;

    QString foldername = tb->gcoor->sgName() + "_" + tb->gcoor->tgName();

    QDir dir(parentFolder);
    dir.mkdir(foldername);

    foreach(Structure::Node * node, tb->scheduler->activeGraph->nodes)
        Synthesizer::saveSynthesisData(node, parentFolder + foldername + "/[activeGraph]");

    foreach(Structure::Node * node, tb->scheduler->targetGraph->nodes)
        Synthesizer::saveSynthesisData(node, parentFolder + foldername + "/[targetGraph]");

    tb->statusBarMessage("Synth data saved.");

    dir.cdUp();
}

void SynthesisManager::loadSynthesisData(QString parentFolder)
{
    if(!tb->blender) return;

    QString foldername = tb->gcoor->sgName() + "_" + tb->gcoor->tgName();
    QDir dir; dir.setCurrent(foldername);

    foreach(Structure::Node * node, tb->scheduler->activeGraph->nodes)
    {
        Synthesizer::clearSynthData(node); // Force load
        Synthesizer::loadSynthesisData(node, parentFolder + foldername + "/[activeGraph]");
    }

    foreach(Structure::Node * node, tb->scheduler->targetGraph->nodes)
    {
        Synthesizer::clearSynthData(node);
        Synthesizer::loadSynthesisData(node, parentFolder + foldername + "/[targetGraph]");
    }

    tb->statusBarMessage("Synth data loaded.");

    tb->drawArea()->updateGL();

    dir.cdUp();
}

void SynthesisManager::doRenderAll()
{
    QElapsedTimer timer; timer.start();

    int stepSize = 1;
    int N = tb->scheduler->allGraphs.size();

    if(tb->scheduler->property.contains("renderCount")){
        int renderCount = tb->scheduler->property["renderCount"].toInt();
        if(renderCount > 1)
            stepSize = double(N) / renderCount;
    }

    int reconLevel = 7;
    if(tb->scheduler->property.contains("reconLevel")){
        reconLevel = tb->scheduler->property["reconLevel"].toInt();
    }

    int startPercentage = tb->scheduler->property["renderStartPercentage"].toInt();
    int startID = N * ( (double) startPercentage / 100);

    for(int i = startID; i < N; i += stepSize)
    {
        Structure::Graph currentGraph = *(tb->scheduler->allGraphs[i]);

        if(i > 0) tb->scheduler->allGraphs[i-1]->clearGeometryCache();

        int progress = (double(i) / (N-1)) * 100;
        qDebug() << QString("Rendering sequence [%1 %]").arg(progress);

        if (progress < 10)
            renderGraph( currentGraph, QString("output_0%1").arg(progress), false, reconLevel );
        else
            renderGraph( currentGraph, QString("output_%1").arg(progress), false, reconLevel );
    }

    qDebug() << QString("Sequence rendered [%1 ms]").arg(timer.elapsed());
}

void SynthesisManager::renderCurrent()
{
    if(!tb->scheduler) return;

    QElapsedTimer timer; timer.start();

    int reconLevel = 7;
    if(tb->scheduler->property.contains("reconLevel")){
        reconLevel = tb->scheduler->property["reconLevel"].toInt();
    }

    tb->setStatusBarMessage(QString("Rendering current frame [depth=%1]..").arg(reconLevel));

    Structure::Graph * currentGraph = tb->graphs.back();

    QDir dir("");
    dir.setCurrent(QFileDialog::getExistingDirectory());

    // Reconstruct
    QString filename = "currentGraph";
    Structure::Graph lastGraph = *currentGraph;
    renderGraph(lastGraph, filename, false, reconLevel, true);

    // Load and display as mesh in viewer
    if( false )
    {
        SurfaceMesh::SurfaceMeshModel * m = new SurfaceMesh::SurfaceMeshModel(filename + ".obj", "reconMesh");
        m->read( qPrintable(filename+".obj") );
        currentGraph->property["reconMesh"].setValue( m );
    }

    tb->updateDrawArea();

    tb->mainWindow()->setStatusBarMessage(QString("Current graph rendered [%1 ms]").arg(timer.elapsed()));
}

void SynthesisManager::renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph )
{
    graph.clearGeometryCache();
    graph.geometryMorph();

    QStringList generatedFiles, tempFiles;

    foreach(Structure::Node * node, graph.nodes)
    {
        // Skip inactive nodes
        if( node->property["zeroGeometry"].toBool() || node->property["shrunk"].toBool() ||
            !node->property.contains("cached_points")) continue;

        QVector<Eigen::Vector3f> points = node->property["cached_points"].value< QVector<Eigen::Vector3f> >();
        QVector<Eigen::Vector3f> normals = node->property["cached_normals"].value< QVector<Eigen::Vector3f> >();

        if(!points.size()) continue;
        std::vector<Eigen::Vector3f> clean_points;
        foreach(Eigen::Vector3f p, points) clean_points.push_back(p);

        std::vector< Eigen::Matrix<float,3,1,Eigen::DontAlign> > finalP, finalN;

        /// Clean up duplicated points
        if( false )
        {
            std::vector<size_t> xrefs;
            weld(clean_points, xrefs, std::hash_Vector3f(), std::equal_to<Eigen::Vector3f>());

            std::set<int> uniqueIds;
            for(int i = 0; i < (int)points.size(); i++)	uniqueIds.insert(xrefs[i]);

            foreach(int id, uniqueIds)
            {
                finalP.push_back( points[id] );
                finalN.push_back( normals[id] );
            }
        }
        else
        {
            foreach(Vector3f p, points) finalP.push_back(p);
            foreach(Vector3f n, normals) finalN.push_back(n);
        }

        /// Better Estimate Normals
        {
            //int num_nighbours = 10;
            //NormalExtrapolation::ExtrapolateNormals(clean_points, normals, num_nighbours);
        }

        /// Smooth normals
        if( true )
        {
            NanoKdTree tree;
            foreach(Vector3f p, finalP) tree.addPoint(p.cast<double>());
            tree.build();

            for(int i = 0; i < (int)finalP.size(); i++)
            {
                Vector3d newNormal(0,0,0);

                int k = 12;

                KDResults matches;
                tree.k_closest(finalP[i].cast<double>(), k, matches);
                foreach(KDResultPair match, matches) newNormal += finalN[match.first].cast<double>();
                newNormal /= 12.0;

                finalN[i] = newNormal.cast<float>();
            }
        }

        /// Send for reconstruction:
        if(isOutPointCloud)
        {
            //QString xyz_filename = node->id + "_" + filename + ".xyz";
            //tempFiles << xyz_filename;
            //Synthesizer::writeXYZ( xyz_filename, finalP, finalN );
        }

        QString node_filename = node->id + ".obj";
        generatedFiles << node_filename;
        PoissonRecon::makeFromCloud( pointCloudf(finalP), pointCloudf(finalN), node_filename, reconLevel );

        // Replace with reconstructed geometries
        if( isOutGraph )
        {
            QFileInfo reconFile( node_filename );

            if( reconFile.exists() )
            {
                SurfaceMesh::Model* nodeMesh = new SurfaceMesh::Model;
                nodeMesh->read( qPrintable(node_filename) );

                node->property["mesh"].setValue( nodeMesh );
                node->property["mesh_filename"].setValue( "meshes/" + node_filename );
            }
        }

        // Clean up
        tb->scheduler->cleanUp();
    }

    combineMeshes(generatedFiles, filename + ".obj");

    if( isOutGraph )
    {
        // Find any removable nodes
        QStringList removableNodes;
        foreach(Node * n, graph.nodes)
        {
            int taskType = n->property["taskType"].toInt();
            bool taskReady = n->property["taskIsReady"].toBool();
            bool taskDone = n->property["taskIsDone"].toBool();
            double t = n->property["t"].toDouble();

            // To-do: cut nodes cases
            {
                // Not yet grown
                if( taskType == Task::GROW && !taskReady && t > 0.0 )
                    removableNodes << n->id;

                // Shrunk nodes
                if( taskType == Task::SHRINK && taskDone )
                    removableNodes << n->id;
            }
        }

        // Remove
        foreach(QString nodeID, removableNodes)
        {
            graph.removeNode( nodeID );
        }

        graph.saveToFile( filename + ".xml" );
    }

    // Clean up
    if(!isOutPointCloud)
    {
        foreach(QString filename, generatedFiles)
            QFile::remove( filename );
    }
}

void SynthesisManager::draftRender()
{
    if(!tb->scheduler) return;

    int stepSize = 1;
    int N = tb->scheduler->allGraphs.size();

    if(tb->scheduler->property.contains("renderCount")){
        int renderCount = tb->scheduler->property["renderCount"].toInt();
        if(renderCount > 1)
            stepSize = double(N) / renderCount;
    }

    if(N < 1) return;

    // Setup camera
    //qglviewer::Camera * camera = drawArea()->camera();

    QString folderPath = QFileDialog::getExistingDirectory();

    for(int i = 0; i < N; i += stepSize)
    {
        tb->graphs.clear();
        tb->graphs.push_back( tb->scheduler->allGraphs[i] );
        tb->drawArea()->updateGL();

        // Save snapshot
        QString snapshotFile;
        int progress = (double(i) / (N-1)) * 100;
        snapshotFile.sprintf("draft_%05d.png", progress);
        snapshotFile = folderPath + "/" + snapshotFile;
        tb->drawArea()->saveSnapshot(snapshotFile, true);

        qDebug() << "File saved: " << snapshotFile;
    }
}

void SynthesisManager::reconstructXYZ()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, "Open XYZ File",
        tb->mainWindow()->settings()->getString("lastUsedDirectory"), "XYZ Files (*.xyz)");
    if(fileNames.isEmpty()) return;

    foreach(QString filename, fileNames)
    {
        PoissonRecon::makeFromCloudFile(filename, filename + ".off", 7);
    }
}

void SynthesisManager::renderAll()
{
    if(!tb->scheduler) return;

    int reconLevel = 7;
    if(tb->scheduler->property.contains("reconLevel")){
        reconLevel = tb->scheduler->property["reconLevel"].toInt();
    }

    tb->setStatusBarMessage(QString("Rendering  requested frames [depth=%1]...").arg(reconLevel));

#ifdef Q_OS_WIN
    QtConcurrent::run( this, &SynthesisManager::doRenderAll );
#else // OpenMP issue on OSX (Linux too?)
    doRenderAll();
#endif

}
