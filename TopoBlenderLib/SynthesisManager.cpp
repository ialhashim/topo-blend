#include <QApplication>
#include <QFileDialog>
#include <QtConcurrentRun>

#include "GlSplatRenderer.h"
GlSplatRenderer * splat_renderer = NULL;

#include "SynthesisManager.h"
#include "QuickMeshDraw.h"

// Synthesis
#include "Synthesizer.h"
#include "normal_extrapolation.h"
#include "SimilarSampling.h"

// Reconstruction
#include "poissonrecon.h"

#include "GraphCorresponder.h"
#include "Scheduler.h"
#include "TopoBlender.h"
#include "Task.h"

// The following depends on the power of the GPU
#define POINTS_LIMIT 600000

QStack<double> nurbsQuality;

Q_DECLARE_METATYPE( std::vector<bool> )
	
SynthesisManager::SynthesisManager( GraphCorresponder * gcorr, Scheduler * scheduler, TopoBlender * blender, int samplesCount ) :
	gcorr(gcorr), scheduler(scheduler), blender(blender), samplesCount(samplesCount), isSplatRenderer(false), 
		splatSize(0.02), pointSize(3), color(QColor::fromRgbF(0.9, 0.9, 0.9))
{
}

void SynthesisManager::clear()
{
	if(scheduler) scheduler->property.remove("synthDataReady");

	synthData.clear();
	renderData.clear();

	sampled.clear();
	vertices.clear();
	currentData.clear();
	currentGraph.clear();

	property["isEnabled"] = false;
}

QVector<Structure::Graph*> SynthesisManager::graphs()
{
	QVector<Structure::Graph*> result;
	result << scheduler->activeGraph << scheduler->targetGraph;
	return result;
}

void SynthesisManager::setSampleCount(int numSamples)
{
	samplesCount = numSamples;
}

void SynthesisManager::genSynData()
{
	clear();

    qApp->setOverrideCursor(Qt::WaitCursor);

    QElapsedTimer timer; timer.start();

    // Number of samples
    uniformTriCount = samplesCount;
	randomCount = uniformTriCount;

	// Progress counter
	int numNodes = scheduler->activeGraph->nodes.size();
	int n = 0;

	// Readability
	Structure::Graph * sgraph = scheduler->activeGraph;
	Structure::Graph * tgraph = scheduler->targetGraph;

    // Generate synthesis data for each corresponding node
    foreach(Structure::Node * snode, sgraph->nodes)
    {
        Structure::Node * tnode = tgraph->getNode( snode->property["correspond"].toString() );

        //int sampling_method = Synthesizer::Random | Synthesizer::Features;
        int sampling_method = Synthesizer::TriUniform | Synthesizer::Features;
        //int sampling_method = Synthesizer::Features;
        //int sampling_method = Synthesizer::Uniform;
        //int sampling_method = Synthesizer::Remeshing;
        //int sampling_method = Synthesizer::Random | Synthesizer::Features | Synthesizer::TriUniform;

		SynthData output;

        if(snode->type() == Structure::CURVE)
        {
            Synthesizer::prepareSynthesizeCurve((Structure::Curve*)snode, (Structure::Curve*)tnode, sampling_method, output);
        }

        if(snode->type() == Structure::SHEET)
        {
            Synthesizer::prepareSynthesizeSheet((Structure::Sheet*)snode, (Structure::Sheet*)tnode, sampling_method, output);
        }

		synthData[sgraph->name()][snode->id] = output["node1"];
		synthData[tgraph->name()][tnode->id] = output["node2"];

        int percent = (double(n++) / (numNodes-1) * 100);
        
		emit( setMessage(QString("Generating data.. [ %1 % ]").arg(percent)) );
		emit( progressChanged(double(percent) / 100.0) );
    }

    QString timingString = QString("Synthesis data [ %1 ms ]").arg(timer.elapsed());
    qDebug() << timingString;
    emit( setMessage(timingString) );

    scheduler->property["synthDataReady"] = true;
	emit( synthDataReady() );

    qApp->restoreOverrideCursor();

	property["isEnabled"] = true;
}

void SynthesisManager::generateSynthesisData()
{
    if(!blender) return;

    setMessage("Generating synthesis samples...");

#ifdef Q_OS_WIN
    QtConcurrent::run( this, &SynthesisManager::genSynData );
#else // OpenMP issue on OSX (Linux too?)
    genSynData();
#endif
}

void SynthesisManager::saveSynthesisData(QString parentFolder)
{
    if(!blender) return;

    QString foldername = gcorr->sgName() + "_" + gcorr->tgName();

    QDir dir(parentFolder);
    dir.mkdir(foldername);

	Structure::Graph * sgraph = scheduler->activeGraph;
	Structure::Graph * tgraph = scheduler->targetGraph;

    foreach(Structure::Node * node, sgraph->nodes)
	{
        Synthesizer::saveSynthesisData(node, parentFolder + foldername + "/[activeGraph]", synthData[sgraph->name()]);
	}

    foreach(Structure::Node * node, tgraph->nodes)
        Synthesizer::saveSynthesisData(node, parentFolder + foldername + "/[targetGraph]", synthData[tgraph->name()]);

    setMessage("Synth data saved.");

    dir.cdUp();
}

void SynthesisManager::loadSynthesisData(QString parentFolder)
{
    if(!blender) return;

    QString foldername = gcorr->sgName() + "_" + gcorr->tgName();
    QDir dir; dir.setCurrent(foldername);

	Structure::Graph * sgraph = scheduler->activeGraph;
	Structure::Graph * tgraph = scheduler->targetGraph;

	int loadedSamples = 0;

    foreach(Structure::Node * node, sgraph->nodes)
    {
        loadedSamples += Synthesizer::loadSynthesisData(node, parentFolder + foldername + "/[activeGraph]", synthData[sgraph->name()]);
    }

    foreach(Structure::Node * node, tgraph->nodes)
    {
        loadedSamples += Synthesizer::loadSynthesisData(node, parentFolder + foldername + "/[targetGraph]", synthData[tgraph->name()]);
    }

	scheduler->property["synthDataReady"] = true;

    setMessage("Synth data loaded.");
	updateViewer();

    dir.cdUp();

	property["isEnabled"] = (loadedSamples > 0);
}

void SynthesisManager::doRenderAll()
{
    QElapsedTimer timer; timer.start();

    int reconLevel = 7;
    if(scheduler->property.contains("reconLevel")){
        reconLevel = scheduler->property["reconLevel"].toInt();
    }

	int N = scheduler->allGraphs.size();

	// Not useful anymore...
	int startPercentage = scheduler->property["renderStartPercentage"].toInt();
	int startID = N * ( (double) startPercentage / 100);

	int renderCount = scheduler->property["renderCount"].toInt();
	int stepSize = qMax(1, N / renderCount);

    for(int i = startID; i < scheduler->allGraphs.size(); i += stepSize)
    {
        Structure::Graph currentGraph = *(scheduler->allGraphs[i]);

        int progress = (double(i) / (N-1)) * 100;
        qDebug() << QString("Rendering sequence [%1 %]").arg(progress);

		QString numString = QString("%1").arg(i, 3, 10, QChar('0'));
        renderGraph( currentGraph, QString("output_%1").arg(numString), false, reconLevel );
    }

    qDebug() << QString("Sequence rendered [%1 ms]").arg(timer.elapsed());
}

void SynthesisManager::renderCurrent()
{
	Structure::Graph * c = currentGraph["graph"].value<Structure::Graph*>();

	if(!c) return;

	renderCurrent(c, "currentGraph");
}

void SynthesisManager::renderCurrent(Structure::Graph * currentGraph, QString path)
{
    if(!scheduler) return;

    QElapsedTimer timer; timer.start();

    int reconLevel = 7;
    if(scheduler->property.contains("reconLevel")){
        reconLevel = scheduler->property["reconLevel"].toInt();
    }

    setMessage(QString("Rendering current frame [depth=%1]..").arg(reconLevel));

	if(!path.size())
	{
		QDir dir("");
		dir.setCurrent(QFileDialog::getExistingDirectory());
	}

    // Reconstruct
	int i = property["curGraphCounter"].toInt();
	property["curGraphCounter"] = i + 1;

	QString filename = "currentGraph_" + QString("%1").arg(i, 3, 10, QChar('0'));
    Structure::Graph lastGraph = *currentGraph;
    renderGraph(lastGraph, path + filename, false, reconLevel, true);

    updateViewer();

    qDebug() << QString("Current graph rendered [%1 ms]").arg(timer.elapsed());
}

void SynthesisManager::renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, 
									int reconLevel, bool isOutGraph /*= false*/, bool isOutParts /*= true */ )
{
	QMap<QString, SurfaceMesh::Model*> reconMeshes;

	renderData.clear();

    geometryMorph(renderData, &graph, false);

	QVector<Node *> usedNodes;

	foreach(Structure::Node * node, graph.nodes){
		// Skip inactive nodes
		if( node->property["zeroGeometry"].toBool() || node->property["shrunk"].toBool()) continue;
		usedNodes.push_back(node);
	}

	// Progress bar
	scheduler->emitProgressStarted();

	for(int ni = 0; ni < (int)usedNodes.size(); ni++)
	{
		int progress = (double(ni) / (usedNodes.size()-1)) * 100;
		scheduler->emitProgressChanged( progress );
		qApp->processEvents();

		Node * node = usedNodes[ni];

        QVector<Eigen::Vector3f> points = renderData[node->id]["points"].value< QVector<Eigen::Vector3f> >();
        QVector<Eigen::Vector3f> normals = renderData[node->id]["normals"].value< QVector<Eigen::Vector3f> >();

        if(!points.size())
		{
			Structure::Node * n = node;

			Vector3 c0 = n->controlPoints().front();
			if( n->property["taskIsDone"].toBool() ){
				n = scheduler->targetGraph->getNode( n->property["correspond"].toString() );
			}

			QSharedPointer<SurfaceMeshModel> nodeMesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();
			Vector3VertexProperty origMeshPoints = nodeMesh->vertex_property<Vector3>(VPOINT);

			Vector3 deltaMesh = n->property["deltaMesh"].value<Vector3>();
			Vector3 v0 = nodeMesh->get_vertex_property<Vector3>(VPOINT)[Vertex(0)];
			Vector3 translation = c0 - (v0 - deltaMesh);

			reconMeshes[node->id] = new SurfaceMesh::Model;

			// Assign a translated copy of the triangle mesh
			foreach(Vertex v, nodeMesh->vertices()) reconMeshes[node->id]->add_vertex( origMeshPoints[v] + translation );
			foreach(Face f, nodeMesh->faces()){
				std::vector<Vertex> verts;
				Surface_mesh::Vertex_around_face_circulator vit = nodeMesh->vertices(f),vend=vit;
				do{ verts.push_back( Vertex(vit) ); } while(++vit != vend);
				reconMeshes[node->id]->add_triangle( verts[0], verts[1], verts[2] );
			}

			if( isOutGraph )
			{
				// Replace node mesh with reconstructed
				QString node_filename = node->id + ".obj";
				node->property["mesh"].setValue( QSharedPointer<SurfaceMeshModel> (reconMeshes[node->id]) );
				node->property["mesh_filename"].setValue( "meshes/" + node_filename );
			}

			continue;
		}

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

		SimpleMesh mesh;
		PoissonRecon::makeFromCloud( pointCloudf(finalP), pointCloudf(finalN), mesh, reconLevel );
		
		reconMeshes[node->id] = new SurfaceMesh::Model;
		SurfaceMesh::Model* nodeMesh = reconMeshes[node->id];

		/// Fill-in reconstructed mesh:
		// Vertices
		for(int i = 0; i < (int)mesh.vertices.size(); i++)
		{
			const std::vector<float> & p = mesh.vertices[i];
			nodeMesh->add_vertex( Vector3(p[0],p[1],p[2]) );
		}
		// Faces
		for(int i = 0; i < (int)mesh.faces.size(); i++)
		{
			std::vector<SurfaceMesh::Vertex> face;
			for(int vi = 0; vi < 3; vi++) face.push_back(SurfaceMesh::Vertex( mesh.faces[i][vi] ));
			nodeMesh->add_face( face );
		}

		assert( mesh.faces.size() != 0 );

		if( isOutGraph )
		{
			// Replace node mesh with reconstructed
			QString node_filename = node->id + ".obj";
			node->property["mesh"].setValue( QSharedPointer<SurfaceMeshModel> (reconMeshes[node->id]) );
			node->property["mesh_filename"].setValue( "meshes/" + node_filename );
		}
    }

	// Write entire reconstructed mesh
	{
		QFile file(filename + ".obj");

		// Create folder
		QFileInfo fileInfo(file.fileName());
		QDir d(""); d.mkpath(fileInfo.absolutePath());

		// Open for writing
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

		QTextStream out(&file);
		int voffset = 0;

		foreach(QString nid, reconMeshes.keys()){
			SurfaceMesh::Model* mesh = reconMeshes[nid];
			out << "# NV = " << mesh->n_vertices() << " NF = " << mesh->n_faces() << "\n";
			SurfaceMesh::Vector3VertexProperty points = mesh->vertex_property<Vector3d>("v:point");

			foreach( SurfaceMesh::Vertex v, mesh->vertices() )
				out << "v " << points[v][0] << " " << points[v][1] << " " << points[v][2] << "\n";
			out << "g " << nid << "\n";
			foreach( SurfaceMesh::Face f, mesh->faces() ){
				out << "f ";
				Surface_mesh::Vertex_around_face_circulator fvit=mesh->vertices(f), fvend=fvit;
				do{	out << (((Surface_mesh::Vertex)fvit).idx() + 1 + voffset) << " ";} while (++fvit != fvend);
				out << "\n";
			}

			voffset += mesh->n_vertices();
		}

		file.close();
	}

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

            // Not yet grown
            if( taskType == Task::GROW && !taskReady && t > 0.0 )
                removableNodes << n->id;

            // Shrunk nodes
            if( taskType == Task::SHRINK && taskDone )
                removableNodes << n->id;
        }

        // Remove
        foreach(QString nodeID, removableNodes)
        {
            graph.removeNode( nodeID );
        }

		// Clean-up names
		foreach(Node * n, graph.nodes) n->id = n->id.replace("_","");
		foreach(Link * e, graph.edges) e->id = e->id.replace("_","");
		
        graph.saveToFile( filename + ".xml", isOutParts );
    }
	else
	{
		// Clean up
		foreach(QString nid, reconMeshes.keys()){
			reconMeshes[nid]->clear();
			delete reconMeshes[nid];
		}
	}

	// Progress bar
	scheduler->emitProgressedDone();
}

void SynthesisManager::reconstructXYZ()
{
    QStringList fileNames = QFileDialog::getOpenFileNames(0, "Open XYZ File","", "XYZ Files (*.xyz)");
    if(fileNames.isEmpty()) return;

    //foreach(QString filename, fileNames)
    //{
    //    PoissonRecon::makeFromCloudFile(filename, filename + ".off", 7);
    //}
}

void SynthesisManager::renderAll()
{
    if(!scheduler) return;

    int reconLevel = 7;
    if(scheduler->property.contains("reconLevel")){
        reconLevel = scheduler->property["reconLevel"].toInt();
    }

    setMessage(QString("Rendering  requested frames [depth=%1]...").arg(reconLevel));

#ifdef Q_OS_WIN
    QtConcurrent::run( this, &SynthesisManager::doRenderAll );
#else // OpenMP issue on OSX (Linux too?)
    doRenderAll();
#endif

}

void SynthesisManager::outputXYZ()
{
	QVector<Structure::Graph*> allGraphs;
	
	allGraphs.push_back(scheduler->activeGraph);
	//allGraphs.push_back(scheduler->targetGraph);

	QVector<Eigen::Vector3f> all_points, all_normals;

	foreach(Structure::Graph * g, allGraphs){
		foreach(Structure::Node * n, g->nodes){
			QVector<Eigen::Vector3f> n_points, n_normals;

			QVector<ParameterCoord> samples = n->property["samples"].value< QVector<ParameterCoord> >();
			QVector<float> offsets = n->property["offsets"].value< QVector<float> >();
			QVector<Vec2f> in_normals = n->property["normals"].value< QVector<Vec2f> >();

			// Without blending!
			if(n->type() == CURVE)
			{
				Curve * curve = (Curve *)n;
				Synthesizer::reconstructGeometryCurve(curve,samples,offsets,in_normals,n_points,n_normals,false);
			}
			if(n->type() == SHEET)
			{
				Sheet * sheet = (Sheet *)n;
				Synthesizer::reconstructGeometrySheet(sheet,samples,offsets,in_normals,n_points,n_normals,false);
			}

			all_points << n_points;
			all_normals << n_normals;
		}
	}

	QFile file("points.xyz");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QTextStream out(&file);

	for(int i = 0; i < (int) all_points.size(); i++)
	{
		Vector3f p = all_points[i];
		Vector3f n = all_normals[i];
		out << QString("%1 %2 %3 %4 %5 %6\n").arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]);
	}

	file.close();
}

Structure::Graph * SynthesisManager::graphNamed( QString graphName )
{
	QMap<QString, Structure::Graph*> allgraphs;
	allgraphs[scheduler->activeGraph->name()] = scheduler->activeGraph;
	allgraphs[scheduler->targetGraph->name()] = scheduler->targetGraph;
	return allgraphs[graphName];
}

void SynthesisManager::drawSampled()
{
	if(!scheduler) return;

	if(!sampled.size() && scheduler->property["synthDataReady"].toBool())
	{
		foreach(Structure::Graph * g, graphs())
		{
			QVector<GLVertex> vertices;

			foreach(Structure::Node * n, g->nodes )
			{
				if(n->id.contains("_null"))
					continue;

				if(!samplesAvailable(g->name(), n->id))
					continue;

				QVector<Vector3f> points, normals;

				if(n->type() == Structure::CURVE){	
					Synthesizer::reconstructGeometryCurve((Structure::Curve *)n,
						synthData[g->name()][n->id]["samples"].value< QVector<ParameterCoord> >(), 
						synthData[g->name()][n->id]["offsets"].value< QVector<float> >(), 
						synthData[g->name()][n->id]["normals"].value< QVector<Vec2f> >(), 
						points, normals, true);
				}

				if(n->type() == Structure::SHEET){	
					Synthesizer::reconstructGeometrySheet((Structure::Sheet *)n,
						synthData[g->name()][n->id]["samples"].value< QVector<ParameterCoord> >(), 
						synthData[g->name()][n->id]["offsets"].value< QVector<float> >(), 
						synthData[g->name()][n->id]["normals"].value< QVector<Vec2f> >(), 
						points, normals, true);
				}

				for(int i = 0; i < (int)points.size(); i++){
					vertices.push_back(	GLVertex(points[i][0], points[i][1], points[i][2],
						normals[i][0], normals[i][1], normals[i][2]) );
				}
			}

			if(!vertices.size()) continue;

			GLuint VertexVBOID;
			glGenBuffers(1, &VertexVBOID);
			glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex) * vertices.size(), &vertices[0].x, GL_STATIC_DRAW);

			sampled[g->name()]["vboID"] = VertexVBOID;
			sampled[g->name()]["count"] = vertices.size();
		}
	}

	foreach(QString key, sampled.keys())
	{
		Structure::Graph * g = graphNamed(key);

		glPushMatrix();

		glTranslated(g->property["posX"].toDouble(), 0, 0);

		glColor4d(1,1,1,0.5);
		glPointSize(2.0);
		glEnable(GL_LIGHTING);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, sampled[key]["vboID"].toUInt());
		glVertexPointer(3, GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, x));
		glNormalPointer(GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, nx));
		glDrawArrays(GL_POINTS, 0, sampled[g->name()]["count"].toInt());

		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);

		glPopMatrix();
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0); // avoid interference with other drawing
}

void SynthesisManager::geometryMorph( SynthData & data, Structure::Graph * graph, bool isApprox, int limit )
{
	Structure::Graph * activeGraph = scheduler->activeGraph;
	Structure::Graph * targetGraph = scheduler->targetGraph;
	QVector<Node*> usedNodes;

	QString ag = activeGraph->name();
	QString tg = targetGraph->name();

	foreach(Node * n, graph->nodes)
	{
		if( n->property["zeroGeometry"].toBool() || n->property["shrunk"].toBool()) 
			continue;

		if(!n->property.contains("correspond"))
			continue;

		// Check against expected skeleton geometry
		{
			Node * origN = scheduler->originalActiveGraph->getNode(n->id);
			QString tid = n->property["corresponded"].toString();

			if( !tid.isEmpty() && n->property["taskIsDone"].toBool() ) origN = scheduler->originalTargetGraph->getNode( tid );

			Array1D_Vector3 cp0 = origN->controlPoints();
			Array1D_Vector3 cp1 = n->controlPoints();
			Vector3 mean0(0,0,0); for(int i = 0; i < (int)cp0.size(); i++) { mean0 += cp0[i]; }; mean0 /= cp0.size();
			Vector3 mean1(0,0,0); for(int i = 0; i < (int)cp1.size(); i++) { mean1 += cp1[i]; }; mean1 /= cp1.size();

			double geoDiff = 0.0;
			for(int i = 0; i < (int)cp0.size(); i++) geoDiff = ((cp0[i]-mean0) - (cp1[i]-mean1)).norm();
			bool isSameGeometry = geoDiff < 1e-8;

			if( isSameGeometry ) continue;
		}

		bool isDeformed = !n->property["fixedSize"].toBool();
		bool isNotDone = n->property.contains("isReady") && n->property.contains("correspond") && n->property["isReady"].toBool() && !n->property["taskIsDone"].toBool();

		if(isDeformed || isNotDone)
		{
			const QVector<float> & offsets = synthData[ag][n->id]["offsets"].value< QVector<float> >();
			if(offsets.isEmpty()) continue;

			usedNodes.push_back(n);
		}
	}

	// Count num samples per node and total
	int numTotalSamples = 0;
	foreach(Node * n, usedNodes){
		const QVector<float> & offsets = synthData[ag][n->id]["offsets"].value< QVector<float> >();
		int numSamples = offsets.size();
		synthData[ag][n->id]["numSamples"] = numSamples;
		numTotalSamples += numSamples;
	}

	foreach(Node * n, usedNodes)
	{
		double t = n->property["localT"].toDouble();

		QString tgnid = n->property["correspond"].toString();

		QVector<Eigen::Vector3f> points, normals;

		SynthData ndata;

		if(limit > 0)
		{
			int numSamplesNode = synthData[ag][n->id]["numSamples"].toInt();
			double relative = double(numSamplesNode) / numTotalSamples;

			// Subsample once
			if( !synthData[ag][n->id].contains("mask") )
			{
				synthData[ag][n->id]["mask"].setValue( subsampleMask(relative * limit, numSamplesNode) );
			
				std::vector<bool> mask = synthData[ag][n->id]["mask"].value< std::vector<bool> >();

				QVector< QVector<ParameterCoord> > nsamples(4);
				QVector< QVector<float> > noffsets(4);
				QVector< QVector<Vec2f> > nnormals(4);

				nsamples[0] = synthData[ag][n->id]["samples"].value< QVector<ParameterCoord> >();
				noffsets[0] = synthData[ag][n->id]["offsets"].value< QVector<float> >();
				nnormals[0] = synthData[ag][n->id]["normals"].value< QVector<Vec2f> >();

				nsamples[1] = synthData[tg][tgnid]["samples"].value< QVector<ParameterCoord> >();
				noffsets[1] = synthData[tg][tgnid]["offsets"].value< QVector<float> >();
				nnormals[1] = synthData[tg][tgnid]["normals"].value< QVector<Vec2f> >();

				for(int i = 0; i < numSamplesNode; i++){
					if(mask[i]){
						nsamples[2].push_back( nsamples[0][i] );
						noffsets[2].push_back( noffsets[0][i] );
						nnormals[2].push_back( nnormals[0][i] );

						nsamples[3].push_back( nsamples[1][i] );
						noffsets[3].push_back( noffsets[1][i] );
						nnormals[3].push_back( nnormals[1][i] );
					}
				}
				synthData[ag][n->id]["subsamples"].setValue( nsamples[2] );
				synthData[ag][n->id]["suboffsets"].setValue( noffsets[2] );
				synthData[ag][n->id]["subnormals"].setValue( nnormals[2] );

				synthData[tg][tgnid]["subsamples"].setValue( nsamples[3] );
				synthData[tg][tgnid]["suboffsets"].setValue( noffsets[3] );
				synthData[tg][tgnid]["subnormals"].setValue( nnormals[3] );
			}

			ndata["node1"]["samples"] = synthData[ag][n->id]["subsamples"];
			ndata["node1"]["offsets"] = synthData[ag][n->id]["suboffsets"];
			ndata["node1"]["normals"] = synthData[ag][n->id]["subnormals"];

			ndata["node2"]["samples"] = synthData[tg][tgnid]["subsamples"];
			ndata["node2"]["offsets"] = synthData[tg][tgnid]["suboffsets"];
			ndata["node2"]["normals"] = synthData[tg][tgnid]["subnormals"];
		}
		else
		{
			ndata["node1"] = synthData[ag][n->id];
			ndata["node2"] = synthData[tg][tgnid];
		}

		if(n->type() == Structure::CURVE) Synthesizer::blendGeometryCurves((Structure::Curve *)n, t, ndata, points, normals, isApprox);
		if(n->type() == Structure::SHEET) Synthesizer::blendGeometrySheets((Structure::Sheet *)n, t, ndata, points, normals, isApprox);

		// Skip bad reconstructed points
		QVector<Eigen::Vector3f> cleanPoints, cleanNormals;
		for(int i = 0; i < points.size(); i++)
		{
			if( std::isnan(points[i][0]) || std::isnan(points[i][1]) || std::isnan(points[i][2]) ) continue;

			cleanPoints.push_back( points[i] );
			cleanNormals.push_back( normals[i] );
		}

		data[n->id]["points"].setValue( cleanPoints );
		data[n->id]["normals"].setValue( cleanNormals );
	}
}

void SynthesisManager::drawSynthesis( Structure::Graph * activeGraph )
{
	// Combine all geometries
	if(currentGraph["graph"].value<Structure::Graph*>() != activeGraph)
	{
		GLuint VertexVBOID;

		// Clean up
		currentGraph.clear();
		currentData.clear();

		if( currentGraph.contains("vboID") ){
			VertexVBOID = currentGraph["vboID"].toUInt();
			glDeleteBuffers(1, &VertexVBOID);
		}

		beginFastNURBS();
		geometryMorph( currentData, activeGraph, true, POINTS_LIMIT );
		endFastNURBS();

		vertices.clear();

		// Fill in GLVertices
		foreach(Node * n, activeGraph->nodes){
			const QVector<Eigen::Vector3f> & points = currentData[n->id]["points"].value< QVector<Eigen::Vector3f> >();
			const QVector<Eigen::Vector3f> & normals = currentData[n->id]["normals"].value< QVector<Eigen::Vector3f> >();
			
			for(int i = 0; i < (int)points.size(); i++){
				vertices.push_back( GLVertex(points[i][0], points[i][1], points[i][2],
					normals[i][0], normals[i][1], normals[i][2]) );
			}
		}
	}

	// Do not reconstruct fixed parts
	foreach(Node * n, activeGraph->nodes)
	{
		const QVector<Eigen::Vector3f> & points = currentData[n->id]["points"].value< QVector<Eigen::Vector3f> >();
		if( points.isEmpty() && property["isEnabled"].toBool() )
		{
			if(!n->property["shrunk"].toBool() && !n->property["zeroGeometry"].toBool())
			{
				Vector3 c0 = n->controlPoints().front();

				if( n->property["taskIsDone"].toBool() ){
					n = scheduler->targetGraph->getNode( n->property["correspond"].toString() );
				}

				QSharedPointer<SurfaceMeshModel> nodeMesh = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >();

				Vector3 deltaMesh = n->property["deltaMesh"].value<Vector3>();

				Vector3 v0 = nodeMesh->get_vertex_property<Vector3>(VPOINT)[Vertex(0)];
				
				Vector3 translation = c0 - (v0 - deltaMesh);

				QuickMeshDraw::drawMeshSolid( nodeMesh.data(), color, translation );
			}
		}

		if(!currentGraph["graph"].value<Structure::Graph*>() && !vertices.size())
			currentGraph["graph"].setValue( activeGraph );
	}

	if(!vertices.size()) 
	{
		activeGraph->property["showMeshes"] = false;
		
		if(!property["isEnabled"].toBool())
		{
			activeGraph->property["showNodes"] = true;
			activeGraph->draw();
		}
		return;
	}

	glEnable(GL_LIGHTING);

	bool isBasicRenderer = !isSplatRenderer;

	if(currentGraph["graph"].value<Structure::Graph*>() != activeGraph)
	{
		if(isBasicRenderer)
		{
			GLuint VertexVBOID = 0;

			glGenBuffers(1, &VertexVBOID);
			glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
			glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex) * vertices.size(), &vertices[0].x, GL_STATIC_DRAW);

			currentGraph["vboID"] = VertexVBOID;
		}
		else
		{
			if(splat_renderer) splat_renderer->update(vertices);
		} 
		
        currentGraph["count"] = (int)vertices.size();
		currentGraph["graph"].setValue( activeGraph );
	}

	if( isBasicRenderer && currentGraph.contains("vboID") )
	{
		glColorQt( color );
		glPointSize( pointSize );

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, currentGraph["vboID"].toUInt());
		glVertexPointer(3, GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, x));
		glNormalPointer(GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, nx));
		glDrawArrays(GL_POINTS, 0, currentGraph["count"].toInt());

		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
	}
	else
	{
		if(!splat_renderer) splat_renderer = new GlSplatRenderer( splatSize );

		splat_renderer->mRadius = splatSize;

		splat_renderer->draw();
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0); // avoid interference with other drawing
}

bool SynthesisManager::samplesAvailable( QString graph, QString nodeID )
{
	return synthData.contains(graph) && 
		synthData[graph].contains(nodeID) &&
		synthData[graph][nodeID].contains("samples") &&
		synthData[graph][nodeID]["samplesCount"].toInt() > 0 ;
}

void SynthesisManager::emitSynthDataReady()
{
	emit( synthDataReady() );
}

void SynthesisManager::bufferCleanup()
{
	GLuint VertexVBOID = this->currentGraph["vboID"].toUInt();
	glDeleteBuffers(1, &VertexVBOID);
	this->currentGraph.clear();
}
