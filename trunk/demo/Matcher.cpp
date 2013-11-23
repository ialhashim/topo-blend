#include "Matcher.h"
using namespace Structure;

typedef QVector< QSet<size_t> > ForcedGroups;
Q_DECLARE_METATYPE( ForcedGroups )

Matcher::Matcher(Scene * scene, QString title) : gcorr(NULL), prevItem(NULL), isAuto(true), DemoPage(scene,title)
{
	// Fill in color sets
	for(int i = 0; i < 40; i++)
	{
		double blueH = 7.0 / 12.0;
		double redH = 0;
		double range = 0.12;

		double coldH = fmod(1.0 + uniformRand( -range + blueH, range + blueH ), 1.0);
		double warmH = fmod(1.0 + uniformRand( -range + redH , range + redH  ), 1.0);

		double coldSat = 0.5;
		double hotSat = 1.0;

		coldColors.push_back(QColor::fromHsvF(coldH, coldSat, 1));
		warmColors.push_back(QColor::fromHsvF(warmH,  hotSat, 1));
	}

	c_cold = c_warm = 0;

	curStrokeColor = QColor(255,0,0,200);

	// Connections
	this->connect(this, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyReleased(QKeyEvent*)));
}

void Matcher::show()
{
	if(!s->isInputReady()) return;

	property.clear();

	QRectF r0 = s->inputGraphs[0]->boundingRect();
	QRectF r1 = s->inputGraphs[1]->boundingRect();

	property["r0"] = r0;
	property["r1"] = r1;

	// Scale in place
	QPointF oldCenter0 = r0.center();
	QPointF oldCenter1 = r1.center();

	double s_factor = (s->width() * 0.5) / r0.width();

	QMatrix m;
	m.scale(s_factor,s_factor);
	r0 = m.mapRect(r0);
	r1 = m.mapRect(r1);
	r0.translate(oldCenter0 - r0.center());
	r1.translate(oldCenter1 - r1.center());
	r0.moveLeft(0);
	r1.moveRight(s->width());

	QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
	animGroup->addAnimation( s->inputGraphs[0]->animateTo(r0) );
	animGroup->addAnimation( s->inputGraphs[1]->animateTo(r1) );
	animGroup->start( QAbstractAnimation::DeleteWhenStopped );
	
	if( gcorr )
	{
		bool isSameSource = gcorr->sg == s->inputGraphs[0]->g;
		bool isSameTarget = gcorr->tg == s->inputGraphs[1]->g;

		if( isSameSource && isSameTarget )
		{
			emit( corresponderCreated(gcorr) );

			manualMode();
			DemoPage::show();

			return;
		}
	}

	// Make corresponder
	gcorr = new GraphCorresponder(s->inputGraphs[0]->g, s->inputGraphs[1]->g);

	// Check for existing correspondence file
	QString path = "dataset/corr/", ext = ".txt";
	QString g1n = s->inputGraphs[0]->name, g2n = s->inputGraphs[1]->name;
	QStringList files; files << (path+g1n+"_"+g2n+ext) << (path+g2n+"_"+g1n+ext);
	int fileidx = -1;
	for(int i = 0; i < 2; i++){
		QString f = files[i];
		if(!QFileInfo (f).exists()) continue;
		fileidx = i;
	}

	if( fileidx != -1 )
	{
		bool corrReversed = (fileidx == 0) ? false : true;

		property["corrFile"] = files[fileidx];
		property["corrReversed"] = corrReversed;

		manualMode();

		// Make sure manual button is selected
		emit( correspondenceFromFile() );
	}
	else
	{
		autoMode();
	}

	// Ordering
	curStrokeColor = QColor(255,0,0);

	// Remove any previous grouping
	s->inputGraphs[0]->g->property.remove("forceGroup");
	s->inputGraphs[1]->g->property.remove("forceGroup");

	emit( corresponderCreated(gcorr) );
	DemoPage::show();
}

void Matcher::hide()
{
	if(!s->isInputReady() || !property.contains("r0")) return;

	// reset colors
	QVector<Structure::Graph*> graphs; 
	graphs << s->inputGraphs[0]->g << s->inputGraphs[1]->g;
	foreach(Structure::Graph * g, graphs){
		foreach(Node * n, g->nodes)
			n->vis_property["meshColor"].setValue( QColor(180,180,180) );
	}

	prevCorrAuto.clear();
	prevCorrManual.clear();

	// Remove any markers
	s->inputGraphs[0]->marker.clear();
	s->inputGraphs[1]->marker.clear();

	// Disable graph picking
	s->setProperty("graph-pick", false);

	QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
	animGroup->addAnimation( s->inputGraphs[0]->animateTo( property["r0"].toRectF() ) );
	animGroup->addAnimation( s->inputGraphs[1]->animateTo( property["r1"].toRectF() ) );
	animGroup->start( QAbstractAnimation::DeleteWhenStopped );

	DemoPage::hide();
}

void Matcher::resetColors()
{
	QSet<QString> snodes, tnodes;

	foreach(Node * n, s->inputGraphs[0]->g->nodes)	snodes.insert(n->id);
	foreach(Node * n, s->inputGraphs[1]->g->nodes)	tnodes.insert(n->id);

	// Skip modified nodes
	foreach(PART_LANDMARK vector2vector, gcorr->correspondences + gcorr->landmarks){
		foreach (QString strID, vector2vector.first) snodes.remove(strID);
		foreach (QString strID, vector2vector.second) tnodes.remove(strID);
	}

	foreach(QString sid, snodes){
		Node * n = s->inputGraphs[0]->g->getNode(sid);
		n->vis_property["meshColor"].setValue( QColor(180,180,180) );
	}

	foreach(QString sid, tnodes){
		Node * n = s->inputGraphs[1]->g->getNode(sid);
		n->vis_property["meshColor"].setValue( QColor(180,180,180) );
	}
}

void Matcher::visualize()
{	
	if( !gcorr ) return; 

	// Clear colors
	resetColors();

	Graph * sourceGraph = s->inputGraphs[0]->g;
	Graph * targetGraph = s->inputGraphs[1]->g;

	// Automatic correspondence
	for(int i = 0; i < (int)gcorr->correspondences.size(); i++)
	{
		PART_LANDMARK vector2vector = gcorr->correspondences[i];

		// Don't change color if same as before
		if(prevCorrAuto.contains(vector2vector)) continue;
		
		QColor curColor = coldColors[c_cold++ % coldColors.size()];

		// User defined correspondence uses warm colors
		if(gcorr->corrScores[vector2vector].front() < 0) 
			curColor = warmColors[c_warm++ % warmColors.size()];

		foreach (QString strID, vector2vector.first)
			sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );

		foreach (QString strID, vector2vector.second)
			targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
	}

	// User specified correspondence
	for(int i = 0; i < (int)gcorr->landmarks.size(); i++)
	{
		PART_LANDMARK vector2vector = gcorr->landmarks[i];

		// Don't change color if same as before
		if(prevCorrManual.contains(vector2vector)) continue;

		QColor curColor = warmColors[c_warm++ % warmColors.size()];

		foreach (QString strID, vector2vector.first)
			sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );

		foreach (QString strID, vector2vector.second)
			targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
	}

	// Non-nodes
	QColor nonColor (180,180,180);
	foreach(int nidx, gcorr->nonCorresS)	sourceGraph->nodes[nidx]->vis_property["meshColor"].setValue( nonColor );
	foreach(int nidx, gcorr->nonCorresT)	targetGraph->nodes[nidx]->vis_property["meshColor"].setValue( nonColor );
	
	prevCorrAuto = gcorr->correspondences;
	prevCorrManual = gcorr->landmarks;
}

void Matcher::graphHit( GraphItem::HitResult hit )
{
	GraphItem * item = hit.item;
	Structure::Graph * g = hit.item->g;
	Structure::Node * n = g->getNode( hit.nodeID );

	QVector<QString> & group = (g == s->inputGraphs[0]->g) ? groupA : groupB;

	// Unique results: add only if not there
	if(!group.contains(n->id)) group.push_back(n->id);

	if( hit.hitMode == MARKING && !isGrouping )
	{
		if(visible)
		{
			double markerRadius = g->bbox().diagonal().norm() * 0.03;
			item->marker.addSphere( hit.ipoint, markerRadius, QColor(211, 50, 118) );
		}
	}

	if( hit.hitMode == STROKES )
	{
		g->property["strokeColor"] = curStrokeColor;

		QVector< QVector<Vector3> > allStorkes = g->property["strokes"].value< QVector< QVector<Vector3> > >();
		
		if(allStorkes.size() < 1) allStorkes.push_back(QVector<Vector3>());

		QVector<Vector3> & lastStroke = allStorkes.back();
		lastStroke.push_back( hit.ipoint );

		g->property["strokes"].setValue( allStorkes );
	}
}

void Matcher::autoMode()
{
	// Disable graph picking
	s->setProperty("graph-pick", false);

	gcorr->clear();
	gcorr->computeCorrespondences();

	prevCorrAuto.clear();
	prevCorrManual.clear();

	// Remove any markers
	s->inputGraphs[0]->marker.clear();
	s->inputGraphs[1]->marker.clear();

	// Visualize computed correspondences
	visualize();

	isAuto = true;
	isGrouping = false;
}

void Matcher::manualMode()
{
	if( property.contains("corrFile") )
	{
		QString filename = property["corrFile"].toString();

		gcorr->clear();
		gcorr->loadCorrespondences( filename, property["corrReversed"].toBool() );

		emit( message( "Correspondence loaded from file: " + filename ) );
	}

	prevCorrAuto.clear();
	prevCorrManual.clear();

	// Enable graph picking
	s->setProperty("graph-pick", true);

	// Visualize computed correspondences
	visualize();

	prevItem = NULL;
	
	isAuto = false;
	isGrouping = false;
}

void Matcher::clearMatch()
{
	groupA.clear();
	groupB.clear();

	s->inputGraphs[0]->marker.clear();
	s->inputGraphs[1]->marker.clear();

	prevItem = NULL;
}

void Matcher::setMatch()
{
	if(!groupA.size() && !groupB.size()) return;

	if(groupA.size() > 0 && groupB.size() > 0)
	{
		gcorr->addLandmarks(groupA, groupB);
	}
	else
	{
		if(groupB.size() == 0)
		{
			foreach(QString nodeID, groupA)
				gcorr->setNonCorresSource( nodeID );
		}
		else
		{
			foreach(QString nodeID, groupB)
				gcorr->setNonCorresTarget( nodeID );
		}
	}

	gcorr->isReady = false;
	gcorr->computeCorrespondences();

	clearMatch();

	visualize();
}

void Matcher::mousePress( QGraphicsSceneMouseEvent* mouseEvent )
{
	if( isAuto && !isGrouping )
	{
		QVector<QRectF> r; 
		r << s->inputGraphs[0]->sceneBoundingRect() << s->inputGraphs[1]->sceneBoundingRect();

		// Test on smaller bounds of input graphs
		int	b = r.front().width() * 0.25;

		for(int i = 0; i < 2; i++){	
			if(r[i].adjusted(b,b,-b,-b).contains(mouseEvent->scenePos())){
				manualMode();
				emit( switchedToManual() );
				return;
			}
		}
	}
}

void Matcher::mouseRelease( QGraphicsSceneMouseEvent* mouseEvent )
{
	Q_UNUSED(mouseEvent)

	if(!visible) return;

	if( isGrouping )
	{
		if(groupA.size() && groupB.size())
		{
			foreach(QString nid, groupA)	s->inputGraphs[0]->g->setColorFor(nid, curStrokeColor);
			foreach(QString nid, groupB)	s->inputGraphs[1]->g->setColorFor(nid, curStrokeColor);

			curStrokeColor = qRandomColor2();

			s->inputGraphs[0]->g->property.remove("strokes");
			s->inputGraphs[1]->g->property.remove("strokes");

			// Convert selection to their unique IDs 
			QSet<size_t> uniqueID;
			foreach(QString nid, groupA)
				uniqueID.insert( size_t(s->inputGraphs[0]->g->getNode(nid)->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data()) );
			foreach(QString nid, groupB)
				uniqueID.insert( size_t(s->inputGraphs[1]->g->getNode(nid)->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data()) );

			// and push them to the list
			ForcedGroups forcedGroups = s->inputGraphs[0]->g->property["forceGroup"].value< ForcedGroups >();
			forcedGroups.push_back(uniqueID);
			s->inputGraphs[0]->g->property["forceGroup"].setValue(forcedGroups);

			groupA.clear();
			groupB.clear();
		}
		else
		{
			QVector<Structure::Graph*> graphs;
			graphs << s->inputGraphs[0]->g << s->inputGraphs[1]->g;

			foreach(Structure::Graph * g, graphs)
				g->property["strokes"].setValue( g->property["strokes"].value< QVector< QVector<Vector3> > >() << QVector<Vector3>() );
		}
	}

	emit( update() );
}

void Matcher::keyReleased( QKeyEvent* keyEvent )
{
	if(!visible) return;

	// Save correspondence
	if(keyEvent->key() == Qt::Key_S){
		QString filename = "dataset/corr/" + (s->inputGraphs[0]->name + "_" + s->inputGraphs[1]->name) + ".txt";
		gcorr->saveCorrespondences(filename, true);
		emit( message( "Correspondence save to file: " + filename ) );
		return;
	}

	if(keyEvent->key() == Qt::Key_R){
		QString filename = "dataset/corr/" + (s->inputGraphs[0]->name + "_" + s->inputGraphs[1]->name) + ".txt";
		QString filename2 = "dataset/corr/" + (s->inputGraphs[1]->name + "_" + s->inputGraphs[0]->name) + ".txt";

		if(QFile::remove(filename) || QFile::remove(filename2)) emit( message( "Correspondence file deleted: " + filename ) );
		return;
	}
}

void Matcher::groupingMode()
{
	// Set to clear color
	QColor clearColor(100,100,100);
	foreach(Node * n, s->inputGraphs[0]->g->nodes)	n->vis_property["meshColor"].setValue( clearColor );
	foreach(Node * n, s->inputGraphs[1]->g->nodes)	n->vis_property["meshColor"].setValue( clearColor );

	// Remove any strokes
	s->inputGraphs[0]->g->property.remove("strokes");
	s->inputGraphs[1]->g->property.remove("strokes");

	// Remove any markers
	s->inputGraphs[0]->marker.clear();
	s->inputGraphs[1]->marker.clear();

	// Show existing groups
	ForcedGroups forcedGroups = s->inputGraphs[0]->g->property["forceGroup"].value< ForcedGroups >();
	
	foreach(QSet<size_t> uids, forcedGroups){
		QColor curGroupColor = qRandomColor2();

		foreach(Structure::Node * n, (s->inputGraphs[0]->g->nodes + s->inputGraphs[1]->g->nodes)){
			size_t uid = size_t(n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data());
			if(uids.contains(uid)) n->vis_property["meshColor"].setValue( curGroupColor );
		}

		curGroupColor = qRandomColor2();
	}

	isGrouping = true;
}
