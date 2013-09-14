#include "Blender.h"
#include "ProgressItem.h"
#include "BlendPathRenderer.h"
#include "BlenderRenderItem.h"

#include "TopoBlender.h"
#include "Scheduler.h"
#include "SynthesisManager.h"

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title), m_gcorr(NULL), s_manager(NULL)
{
	this->numSuggestions = 4;
	this->numInBetweens = 4;

	// Create background items for each blend path
	int padding = 0;
	int blendPathHeight = (s->height() / (numSuggestions * 1.5)) * 0.99;

	int totalHeight = numSuggestions * (blendPathHeight + padding);
	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 45;

	for(int i = 0; i < numSuggestions; i++)
	{
		QGraphicsRectItem * blendPathBack = new QGraphicsRectItem (0,0,s->width() * 0.5, blendPathHeight);
		blendPathBack->setBrush( QColor::fromRgbF(0,0,0.25,0.5) );
		blendPathBack->setOpacity(0);

		blendPathBack->setY( startY + (i * (blendPathHeight + padding)) );
		blendPathBack->setX( 0.5*s->width() - 0.5*blendPathBack->boundingRect().width() );
		blendPathBack->setZValue(-999);

		QGraphicsItemGroup * blendPathItem = new QGraphicsItemGroup;
		blendPathItem->addToGroup(blendPathBack);
		blendPathItem->setVisible(false);

		blendPathsItems.push_back( blendPathItem );
		scene->addItem(blendPathItem);
	}

	// Show path indicators
	QRectF r0(0,0,s->width() * 0.25,s->width() * 0.25);	r0.moveTop((s->height() * 0.5) - (r0.height() * 0.5));
	QRectF r1 = r0; r1.moveRight(s->width());
	QPointF lstart = r0.center() + QPoint(r0.width() * 0.3,0);
	QPointF rstart = r1.center() - QPoint(r1.width() * 0.3,0);
	double tension = 0.5;

	for(int i = 0; i < numSuggestions; i++)
	{
		QRectF r = blendPathsItems[i]->boundingRect();
		QPointF lend = QPointF(r.x(), r.center().y());
		QPointF rend = QPointF(r.x() + r.width(), r.center().y());
		double lmidX = (lend.x() - lstart.x()) * tension;
		double rmidX = (rstart.x() - rend.x()) * tension;

		QVector<QPointF> l_points, r_points;

		l_points.push_back(lstart);
		l_points.push_back(lstart + QPointF(lmidX,0));
		l_points.push_back(lend - QPointF(lmidX,0));
		l_points.push_back(lend);

		r_points.push_back(rstart);
		r_points.push_back(rstart - QPointF(rmidX,0));
		r_points.push_back(rend + QPointF(rmidX,0));
		r_points.push_back(rend);

		QPainterPath lpath;
		lpath.moveTo(l_points.at(0));
		{
			int i=1;
			while (i + 2 < l_points.size()) {
				lpath.cubicTo(l_points.at(i), l_points.at(i+1), l_points.at(i+2));
				i += 3;
			}
		}

		QPainterPath rpath;
		rpath.moveTo(r_points.at(0));
		{
			int i=1;
			while (i + 2 < r_points.size()) {
				rpath.cubicTo(r_points.at(i), r_points.at(i+1), r_points.at(i+2));
				i += 3;
			}
		}

		QColor color( 255, 180, 68, 50 );

		QGraphicsItem * litem = s->addPath(lpath, QPen(color, 1));
		QGraphicsItem * ritem = s->addPath(rpath, QPen(color, 1));

		litem->setVisible(false);
		ritem->setVisible(false);

		items.push_back(litem);
		items.push_back(ritem);
	}

	for(int i = 0; i < blendPathsItems.size(); i++)
		items.push_back( blendPathsItems[i] );

	itemHeight = blendPathHeight;

	// Progress bar
	progress = new ProgressItem("Working..", false, s);

	// Connections
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

	// Give time for animation
	progress->show();
	QTimer::singleShot(300, this, SLOT(preparePaths()));
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

		// Add blend path
		blendPaths.push_back( bp );
		
		// Synthesis requires a single instance of the blend process
		if( !s_manager )
		{
			int numSamples = 5000;

			#ifdef QT_DEBUG
				numSamples = 50;
			#endif

			s_manager = new SynthesisManager(m_gcorr, bp.scheduler, bp.blender, numSamples);

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
			double start = 1.0 / (numInBetweens+2);
			double range = 1.0 - (2.0 * start);

			double t = ((double(j) / (numInBetweens-1)) * range) + start;

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

	qDebug() << pathID;

	int delta = 0.5 * ( (pathRect.width() / numInBetweens) - item->boundingRect().width() );

	int x = pathRect.x() + (blendIDX * (pathRect.width() / numInBetweens)) + delta;
	int y = pathRect.y();

	item->setPos(x, y);
}

void Blender::cleanUp()
{
	for(int i = 0; i < resultItems.size(); i++){
		s->removeItem(resultItems[i]);
		delete resultItems[i];
	}

	// Clean up blending path data
	{
		blendPaths.clear();
	}

	// Clean up synthesis data
	{
		s_manager->clear();
		delete s_manager;
		s_manager = NULL;
	}

	resultItems.clear();
}
