#include "Matcher.h"
using namespace Structure;

Matcher::Matcher(Scene * scene, QString title) : prevItem(NULL), DemoPage(scene,title)
{
	// Fill in color sets
	for(int i = 0; i < 20; i++)
	{
		double blueH = 7.0 / 12.0;
		double redH = 0;
		double range = 0.1;

		double coldH = fmod(1.0 + uniformRand( -range + blueH, range + blueH ), 1.0);
		double warmH = fmod(1.0 + uniformRand( -range + redH , range + redH  ), 1.0);

		double coldSat = 0.5;
		double hotSat = 1.0;

		coldColors.push_back(QColor::fromHsvF(coldH, coldSat, 1));
		warmColors.push_back(QColor::fromHsvF(warmH,  hotSat, 1));
	}

	c_cold = c_warm = 0;
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

	emit( corresponderCreated(gcorr) );

	DemoPage::show();
}

void Matcher::resetColors()
{
	QVector<Structure::Graph*> graphs; 
	graphs << s->inputGraphs[0]->g << s->inputGraphs[1]->g;
	foreach(Structure::Graph * g, graphs){
		foreach(Node * n, g->nodes)
			n->vis_property["meshColor"].setValue( QColor(180,180,180) );
	}
}

void Matcher::hide()
{
	if(!s->isInputReady() || !property.contains("r0")) return;

	resetColors();

	// Disable graph picking
	s->setProperty("graph-pick", false);

	QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
	animGroup->addAnimation( s->inputGraphs[0]->animateTo( property["r0"].toRectF() ) );
	animGroup->addAnimation( s->inputGraphs[1]->animateTo( property["r1"].toRectF() ) );
	animGroup->start( QAbstractAnimation::DeleteWhenStopped );

	DemoPage::hide();
}

void Matcher::visualize()
{
	// Clear colors
	resetColors();

	c_cold = 0;
	c_warm = 0;

	// Color previously assigned correspondences
	if( gcorr ) {
		Graph * sourceGraph = s->inputGraphs[0]->g;
		Graph * targetGraph = s->inputGraphs[1]->g;

		int i = 0;

		// Automatic correspondence
		foreach (PART_LANDMARK vector2vector, gcorr->correspondences){
			QColor curColor = coldColors[c_cold++ % coldColors.size()];

			// User defined correspondence uses warm colors
			if(gcorr->corrScores[i++].front() < 0) curColor = warmColors[c_warm++ % warmColors.size()];

			foreach (QString strID, vector2vector.first)
				sourceGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );

			foreach (QString strID, vector2vector.second)
				targetGraph->getNode( strID )->vis_property["meshColor"].setValue( curColor );
		}

		// User specified correspondence
		foreach (PART_LANDMARK vector2vector, gcorr->landmarks){
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
	}
}

void Matcher::graphHit( GraphItem::HitResult hit )
{
	GraphItem * item = hit.item;
	Structure::Graph * g = hit.item->g;
	Structure::Node * n = g->getNode( hit.nodeID );

	QVector<QString> & group = (g == s->inputGraphs[0]->g) ? groupA : groupB;
	
	group.push_back(n->id);
	item->marker.addSphere( hit.ipoint, 0.04, QColor(211, 50, 118) );
}

void Matcher::autoMode()
{
	// Disable graph picking
	s->setProperty("graph-pick", false);

	gcorr->clear();
	gcorr->computeCorrespondences();

	// Remove any markers
	s->inputGraphs[0]->marker.clear();
	s->inputGraphs[1]->marker.clear();

	// Visualize computed correspondences
	visualize();
}

void Matcher::manualMode()
{
	if( property.contains("corrFile") )
	{
		gcorr->clear();
		gcorr->loadCorrespondences( property["corrFile"].toString(), property["corrReversed"].toBool() );
		gcorr->isReady = true;
	}

	// Enable graph picking
	s->setProperty("graph-pick", true);

	// Visualize computed correspondences
	visualize();

	prevItem = NULL;
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
