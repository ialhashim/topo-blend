#include "correspondence-manager.h"
#include "StructureGraph.h"
#include "GraphCorresponder.h"

using namespace Structure;

GraphCorresponder* CorrespondenceManager::makeCorresponder()
{
	if (tb->graphs.size() < 2)
	{
		qDebug() << "Please load at least two graphs.";
		return NULL;
	}

	Structure::Graph *sg = tb->graphs[0];
	Structure::Graph *tg = tb->graphs[1];

	tb->gcoor = new GraphCorresponder(sg, tg);

	return tb->gcoor;
}

void CorrespondenceManager::assignCorrespondence()
{
	Graph * sourceGraph = tb->graphs.front();
	Graph * targetGraph = tb->graphs.back();

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

	QColor curColor = qRandomColor3(0, 0.25);

	if(!tb->gcoor) tb->gcoor = makeCorresponder();

	// Correspondence specified
	if(sParts.size() > 0 && tParts.size() > 0)
	{
		tb->gcoor->addLandmarks(sParts, tParts);

		// Assign colors
		foreach(QString nodeID, sParts)
			sourceGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );
		
		foreach(QString nodeID, tParts)	
			targetGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );

		tb->gcoor->isReady = false;
	}
	else
	{
		if(tParts.size() == 0)
		{
			foreach(QString nodeID, sParts){
				tb->gcoor->setNonCorresSource( nodeID );
				sourceGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );
			}
		}
		else
		{
			foreach(QString nodeID, tParts){
				tb->gcoor->setNonCorresTarget( nodeID );
				targetGraph->getNode( nodeID )->vis_property["meshColor"].setValue( curColor );
			}
		}

		tb->gcoor->isReady = false;
	}
}

void CorrespondenceManager::exitCorrespondenceMode(bool isChangeVisualization)
{
	tb->drawArea()->setMouseBinding(Qt::LeftButton, QGLViewer::CAMERA, QGLViewer::ROTATE);
	tb->drawArea()->setMouseBinding(Qt::SHIFT | Qt::LeftButton, QGLViewer::SELECT);

	// Reset mesh visualization
	if( isChangeVisualization )
	{
		foreach(Graph * g, tb->graphs){
			foreach(Node * n, g->nodes){
				n->vis_property["meshSolid"] = false;
				n->vis_property["meshColor"].setValue( QColor(200,200,200,8) );
			}
		}
	}

	tb->property["correspondenceMode"] = false;

	tb->drawArea()->updateGL(); 
}

void CorrespondenceManager::visualizeAsSolids()
{
	// mesh visualization - set to solid gray
	foreach(Graph * g, tb->graphs){
		foreach(Node * n, g->nodes){
			n->vis_property["meshSolid"] = true;
			n->vis_property["meshColor"].setValue( QColor(180,180,180) );
		}
	}
}

void CorrespondenceManager::correspondenceMode()
{
	// Enter / exit correspondence mode
	tb->property["correspondenceMode"] = !tb->property["correspondenceMode"].toBool();

	if(!tb->property["correspondenceMode"].toBool()) 
	{ 
		exitCorrespondenceMode(true);
		return; 
	}

	tb->drawArea()->setMouseBinding(Qt::LeftButton, QGLViewer::SELECT);
	tb->drawArea()->setMouseBinding(Qt::SHIFT | Qt::LeftButton, QGLViewer::CAMERA, QGLViewer::ROTATE);

	this->visualizeAsSolids();

	// Color previously assigned correspondences
	if( tb->gcoor ) {
		Graph * sourceGraph = tb->graphs.front();
		Graph * targetGraph = tb->graphs.back();

		foreach (PART_LANDMARK vector2vector, tb->gcoor->correspondences){
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

		foreach (PART_LANDMARK vector2vector, tb->gcoor->landmarks){
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
}

void CorrespondenceManager::clearCorrespondence()
{
	foreach(Graph * g, tb->graphs){
		foreach(Node * n, g->nodes){
			n->vis_property["meshColor"].setValue( QColor(180,180,180) );
			n->property["is_corresponded"] = false;
		}
	}

	if(tb->gcoor) tb->gcoor->clear();
}

void CorrespondenceManager::drawWithNames()
{
	int offset = 0;

	for(int gID = 0; gID < (int) tb->graphs.size(); gID++)
	{
		Structure::Graph *g = tb->graphs[gID];

		glPushMatrix();
		glTranslated(g->property["posX"].toDouble(), 0, 0);
		g->drawNodeMeshNames( offset );
		glPopMatrix();
	}
}
