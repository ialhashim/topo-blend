#include "Blender.h"
#include "ProgressItem.h"
#include "BlendPathRenderer.h"
#include "BlenderRenderItem.h"

#include "TopoBlender.h"
#include "Scheduler.h"
#include "SynthesisManager.h"

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title), m_gcorr(NULL), s_manager(NULL)
{
	this->numSuggestions = 5;
	this->numInBetweens = 5;

	// Create background items for each blend path
	int padding = 5;
	int blendPathHeight = (s->height() / (numSuggestions * 1.5)) * 0.98;

	int totalHeight = numSuggestions * (blendPathHeight + padding);
	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 45;

	for(int i = 0; i < numSuggestions; i++)
	{
		QGraphicsItem * blendPathBack = scene->addRect(0,0,s->width() * 0.5, blendPathHeight, QPen(), QColor::fromRgbF(0,0,0.25,0.5) );

		blendPathBack->setY( startY + (i * (blendPathHeight + padding)) );
		blendPathBack->setX( 0.5*s->width() - 0.5*blendPathBack->boundingRect().width() );

		blendPathBack->setZValue(-999);
		blendPathBack->setVisible(false);

		QGraphicsItemGroup * blendPathItem = new QGraphicsItemGroup;
		blendPathItem->addToGroup(blendPathBack);
		scene->addItem(blendPathItem);

		blendPathsItems.push_back( blendPathItem );
	}

	for(int i = 0; i < blendPathsItems.size(); i++)
		items.push_back( blendPathsItems[i] );

	itemHeight = blendPathHeight;

	// Progress bar
	progress = new ProgressItem("Working..", false, s);

	// Connections
	this->connect(this, SIGNAL(becameVisible()), SLOT(preparePaths()));
	this->connect(this, SIGNAL(blendPathsReady()), SLOT(computeBlendPaths()));
	this->connect(this, SIGNAL(allPathsDone()), SLOT(blenderDone()));
	this->connect(this, SIGNAL(becameHidden()), SLOT(cleanUp()));
}

void Blender::show()
{
    if(!s->isInputReady()) return;

    QRectF r0 = s->inputGraphs[0]->boundingRect();
    QRectF r1 = s->inputGraphs[1]->boundingRect();

    property["r0"] = r0;
    property["r1"] = r1;

    // Scale in place
    QPointF oldCenter0 = r0.center();
    QPointF oldCenter1 = r1.center();

    double s_factor = (s->width() * 0.25) / r0.width();

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

    DemoPage::show();

	qApp->processEvents();
}

void Blender::hide()
{
    if(!s->isInputReady() || !property.contains("r0")) return;

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo( property["r0"].toRectF() ) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo( property["r1"].toRectF() ) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );

    DemoPage::hide();
}

void Blender::setGraphCorresponder( GraphCorresponder * graphCorresponder )
{
	this->m_gcorr = graphCorresponder;
}

void Blender::preparePaths()
{    
	if(!s->isInputReady() || m_gcorr == NULL) return;
	
	qApp->setOverrideCursor(Qt::WaitCursor);
	qApp->processEvents();

	// Generate blend paths
	for(int i = 0; i < numSuggestions; i++)
	{
		BlendPath bp;

		bp.source = s->inputGraphs[0]->g;
		bp.target = s->inputGraphs[1]->g;
		bp.gcorr = this->m_gcorr;

		bp.scheduler = new Scheduler;
		bp.blender = new TopoBlender( bp.gcorr, bp.scheduler );

		/// Different scheduling happens here...
		if(i != 0) bp.scheduler->shuffleSchedule();

		this->connect(bp.scheduler, SIGNAL(progressChanged(int)), SLOT(progressChanged()));
		this->connect(bp.scheduler, SIGNAL(progressDone()), SLOT(pathDone()));

		blendPaths.push_back( bp );
		
		// Synthesis requires a single instance of the blend process
		if( !s_manager )
		{
			s_manager = new SynthesisManager(m_gcorr, bp.scheduler, bp.blender, 5000);

			QVariant p_camera;
			p_camera.setValue( s->camera );
			s_manager->setProperty("camera", p_camera);

			// Progress connections
			progress->connect(s_manager, SIGNAL(progressChanged(double)), SLOT(setProgress(double)));
			this->connect(s_manager, SIGNAL(synthDataReady()), SLOT(synthDataReady()));
		}
	}

	qApp->restoreOverrideCursor();

	// Generate samples
	progress->startProgress();
	progress->setExtra("Generating samples - ");
	s_manager->generateSynthesisData();
}

void Blender::synthDataReady()
{
	emit( blendPathsReady() );
}

void Blender::computePath( int index )
{
	blendPaths[index].scheduler->doBlend();
	//blendPaths[index].blender->setupUI();
}

void Blender::computeBlendPaths()
{
	progress->startProgress();
	progress->setExtra("Blend paths - ");

	for(int i = 0; i < blendPaths.size(); i++)
	{
		computePath( i );
	}
}

void Blender::progressChanged()
{
	int curProgress = 0;
	for(int i = 0; i < blendPaths.size(); i++){
		curProgress += blendPaths[i].scheduler->property["progress"].toInt();
	}

	int totalProgress = 100 * blendPaths.size();

	progress->setProgress( double(curProgress) / totalProgress );
}

void Blender::pathDone()
{
	int numDone = 0;
	for(int i = 0; i < blendPaths.size(); i++)
		if(blendPaths[i].scheduler->property["progressDone"].toBool())
			numDone++;
	if(numDone == blendPaths.size()) emit( allPathsDone() );
}

void Blender::blenderDone()
{
	progress->stopProgress();
	progress->hide();

	// Draw results
	BlendPathRenderer * renderer = new BlendPathRenderer(s_manager, itemHeight);
	this->connect( renderer, SIGNAL(itemReady(QGraphicsItem*)), SLOT(blendResultDone(QGraphicsItem*)) );

	for(int i = 0; i < numSuggestions; i++)
	{
		Scheduler * curSchedule = blendPaths[i].scheduler;

		for(int j = 0; j < numInBetweens; j++)
		{
			double t = double(j) / (numInBetweens - 1);
			int idx = t * (curSchedule->allGraphs.size() - 1);
			Structure::Graph * curGraph = curSchedule->allGraphs[idx];

			renderer->generateItem( curGraph, i, j );
		}
	}
}

void Blender::blendResultDone(QGraphicsItem* done_item)
{
	BlenderRenderItem * item = (BlenderRenderItem*) done_item;

	int pathID = item->pathID;
	int blendIDX = item->blendIDX;

	resultItems.push_back(item);
	//items.push_back(item);
	s->addItem(item);

	QRectF pathRect = blendPathsItems[pathID]->boundingRect();

	int x = pathRect.x() + (blendIDX * (pathRect.width() / numInBetweens));
	int y = pathRect.y();

	item->setPos(x, y);
}

void Blender::cleanUp()
{
	for(int i = 0; i < resultItems.size(); i++){
		s->removeItem(resultItems[i]);
		delete resultItems[i];
	}
	resultItems.clear();
}
