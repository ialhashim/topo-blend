#include "Blender.h"
#include "ProgressItem.h"
#include "BlendRenderItem.h"
#include "BlendPathRenderer.h"
#include "BlendPathSub.h"

#include "TopoBlender.h"
#include "Scheduler.h"
#include "SynthesisManager.h"

QVector< BlendPath > blendPaths;
QList< QSharedPointer<Scheduler> > jobs;

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title), m_gcorr(NULL), s_manager(NULL)
{
	this->numSuggestions = 4;
	this->numInBetweens = 4;

	this->resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
	this->blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens) );

	this->isSample = true;
#ifdef QT_DEBUG
	this->isSample = false;
#endif

	this->graphItemWidth = s->width() * 0.2;

	setupBlendPathItems();

	// Results renderer
	renderer = new BlendPathRenderer(this, itemHeight);
	this->connect( renderer, SIGNAL(itemReady(QGraphicsItem*)), SLOT(blendResultDone(QGraphicsItem*)), Qt::DirectConnection );

	// Progress bar
	progress = new ProgressItem("Working..", false, s);
	progress->setZValue(999);

	// Connections
    this->connect(this, SIGNAL(blendPathsReady()), SLOT(runComputeBlendPaths()));
    this->connect(this, SIGNAL(blendPathsDone()), SLOT(blenderDone()));
	this->connect(this, SIGNAL(becameHidden()), SLOT(cleanUp()));
	this->connect(this, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyReleased(QKeyEvent*)));
	this->connect(this, SIGNAL(blendDone()), SLOT(blenderAllResultsDone()));
}

void Blender::setupBlendPathItems()
{
	// Create background items for each blend path
	int padding = 4;
	int blendPathHeight = (s->height() / (numSuggestions * 1.5)) * 0.99;
	int totalHeight = numSuggestions * (blendPathHeight + padding);
	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 45;

	for(int i = 0; i < numSuggestions; i++)
	{
		QGraphicsRectItem * blendPathBack = new QGraphicsRectItem (0,0,s->width() - (2 * graphItemWidth), blendPathHeight);
		blendPathBack->setBrush( QColor (255, 180, 68) );
		blendPathBack->setOpacity( 0.04 );
		blendPathBack->setY( startY + (i * (blendPathHeight + padding)) );
		blendPathBack->setX( 0.5*s->width() - 0.5*blendPathBack->boundingRect().width() );

		QGraphicsItemGroup * blendPathItem = new QGraphicsItemGroup;
		blendPathItem->addToGroup(blendPathBack);
		blendPathItem->setVisible(false);
		blendPathItem->setZValue(-999);

		blendPathsItems.push_back( blendPathItem );
		s->addItem(blendPathItem);
		items.push_back(blendPathItem);
	}

	// Show path indicators
	QRectF r0(0,0,graphItemWidth,graphItemWidth);	r0.moveTop((s->height() * 0.5) - (r0.height() * 0.5));
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

		QColor color( 255, 180, 68, 100 );
		QGraphicsItem * litem = s->addPath(lpath, QPen(color, 1));
		QGraphicsItem * ritem = s->addPath(rpath, QPen(color, 1));
		litem->setVisible(false);
		ritem->setVisible(false);

		auxItems.push_back(litem);
		auxItems.push_back(ritem);
	}

	itemHeight = blendPathHeight;
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
    double s_factor = graphItemWidth / r0.width();
    QMatrix m;
    m.scale(s_factor,s_factor);
    r0 = m.mapRect(r0);
    r1 = m.mapRect(r1);
    r0.translate(oldCenter0 - r0.center());
    r1.translate(oldCenter1 - r1.center());
    r0.moveLeft(0);
    r1.moveRight(s->width());

	// Move input shapes to edge of screen
    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo(r0) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo(r1) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );

	DemoPage::show();

	// Give time for animations to finish
	QTimer::singleShot(300, this, SLOT(preparePaths()));
}

void Blender::hide()
{
    if(!s->isInputReady() || !property.contains("r0")) return;

	foreach(QGraphicsItem * item, auxItems) item->setVisible(false);

	// Return the two input shapes to original state
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

	isFinished = false;
	emit( blendStarted() );

	// UI and logging
	{	
		progress->setExtra("Preparing paths - ");
		progress->show();

		qApp->setOverrideCursor(Qt::WaitCursor);
		qApp->processEvents();
		pathsTimer.start();
	}

	// Generate blend paths
	for(int i = 0; i < numSuggestions; i++)
	{
		BlendPath bp;

		bp.source = s->inputGraphs[0]->g;
		bp.target = s->inputGraphs[1]->g;
		bp.gcorr = this->m_gcorr;

		// Per-path data
		bp.scheduler = QSharedPointer<Scheduler>( new Scheduler );
		bp.blender = QSharedPointer<TopoBlender>( new TopoBlender( bp.gcorr, bp.scheduler.data() ) );

		/// Different scheduling happens here...
		if(i != 0) bp.scheduler->shuffleSchedule();

		// Add blend path
		blendPaths.push_back( bp );

		// Connections
        this->connect(bp.scheduler.data(), SIGNAL(progressChanged(int)), SLOT(pathProgressChanged()), Qt::DirectConnection);

		// Synthesis requires a single instance of the blend process
		if( s_manager.isNull() )
		{
			int numSamples = 8000;

			s_manager = QSharedPointer<SynthesisManager>(new SynthesisManager(m_gcorr, bp.scheduler.data(), bp.blender.data(), numSamples));

			QVariant p_camera;
			p_camera.setValue( s->camera );
			s_manager->setProperty("camera", p_camera);

			// Progress connections
			progress->connect(s_manager.data(), SIGNAL(progressChanged(double)), SLOT(setProgress(double)));
			this->connect(s_manager.data(), SIGNAL(synthDataReady()), SLOT(synthDataReady()));
		}
	}

	// UI and logging
	{
		qApp->restoreOverrideCursor();
		progress->startProgress();
		progress->setExtra("Generating samples - ");
		emit( message(QString("Paths time [%1 ms]").arg(pathsTimer.elapsed())) );
	}

	synthTimer.start();

	// [HEAVY] Generate synthesis data
	if( isSample )
		s_manager->generateSynthesisData();
	else
		emit( s_manager->emitSynthDataReady() );
}

void Blender::synthDataReady()
{
	for(int i = 0; i < numSuggestions; i++)
		this->disconnect( blendPaths[i].scheduler.data() );

	emit( blendPathsReady() );
	emit( message(QString("Synthesis time [%1 ms]").arg(synthTimer.elapsed())) );
}

void Blender::runComputeBlendPaths()
{
	progress->startProgress();
	progress->setExtra("Blend paths - ");
	blendTimer.start();

    QtConcurrent::run( this, &Blender::computeBlendPaths );
}

void executeJob( const QSharedPointer<Scheduler> & scheduler )
{
#ifdef QT_DEBUG
	scheduler->timeStep = 0.2;
#endif

	scheduler->executeAll();
}

void Blender::computeBlendPaths()
{
    jobs.clear();
    for(int i = 0; i < blendPaths.size(); i++) jobs.push_back( blendPaths[i].scheduler );

    QFuture<void> future = QtConcurrent::map(jobs, executeJob);
    future.waitForFinished();

    emit( blendPathsDone() );
}

void Blender::computePath( const int & index )
{
    blendPaths[index].scheduler->executeAll();
}

void Blender::pathProgressChanged()
{
	int curProgress = 0;
	for(int i = 0; i < blendPaths.size(); i++){
		curProgress += blendPaths[i].scheduler->property["progress"].toInt();
	}
	int totalProgress = 100 * blendPaths.size();

	progress->setProgress( double(curProgress) / totalProgress );
}

void Blender::blenderDone()
{
	// UI and logging
	{
        emit( message(QString("Blending time [%1 ms]").arg(blendTimer.elapsed())) );

		progress->setExtra("Rendering -");
		progress->setProgress(0.0);
		renderTimer.start();
	}

	// For each blend path, render 'k' in-betweens
	for(int i = 0; i < numSuggestions; i++)
	{
		Scheduler * curSchedule = blendPaths[i].scheduler.data();

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

	foreach(QGraphicsItem * item, auxItems) item->setVisible(true);
}

void Blender::keyReleased( QKeyEvent* keyEvent )
{
	if(keyEvent->key() == Qt::Key_F)
	{
		this->isSample = !this->isSample;
		emit( message( QString("Sampling toggled to: %1").arg(isSample) ) );
	}
}

void Blender::blendResultDone(QGraphicsItem* done_item)
{
	BlendRenderItem * item = (BlendRenderItem*) done_item;

	int pathID = item->property["pathID"].toInt();
	int blendIDX = item->property["blendIDX"].toInt();

	resultItems[pathID][blendIDX] = QSharedPointer<BlendRenderItem>(item);

	s->addItem( resultItems[pathID][blendIDX].data() );

	// Placement
	QRectF pathRect = blendPathsItems[pathID]->boundingRect();
	double outterWidth = (pathRect.width() / numInBetweens);
	int delta = 0.5 * (outterWidth  - item->boundingRect().width() );
	int x = pathRect.x() + (blendIDX * outterWidth) + delta;
	int y = pathRect.y();
	item->setPos(x, y);

	// UI and logging
	{
		int numDone = 0;
		for(int i = 0; i < numSuggestions; i++)
			for(int j = 0; j < numInBetweens; j++)
				if(resultItems[i][j]) numDone++;

		int N = (numSuggestions * numInBetweens);
		progress->setProgress( double(numDone) / N );

		if(numDone == N)
		{
			isFinished = true;
			emit( blendDone() );

			progress->stopProgress();
			progress->hide();
			emit( message(QString("Render time [%1 ms]").arg(renderTimer.elapsed())) );
		}

		qApp->processEvents();
	}
}

void Blender::cleanUp()
{
	// Clear results
	{
		resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
		blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens) );
	}
	
	// Clean up synthesis data
	{
		s_manager.clear();
	}

	// Clean up blending path data
	{
		blendPaths.clear();
	}
}

void Blender::blenderAllResultsDone()
{
	// Expand blend sequence near two items
	for(int i = 0; i < numSuggestions; i++)
	{
		QRectF pathRect = blendPathsItems[i]->boundingRect();
		double outterWidth = (pathRect.width() / numInBetweens);

		for(int j = 0; j + 1 < numInBetweens; j++)
		{
			BlendPathSubButton * subItemButton = new BlendPathSubButton(itemHeight * 0.5, itemHeight, this);
			subItemButton->setPos( QPointF(pathRect.x() + ((j+1) * outterWidth) - (subItemButton->boundingRect().width() * 0.5), pathRect.y()) );
			
			blendSubItems[i][j] = QSharedPointer<BlendPathSubButton>(subItemButton);

			blendSubItems[i][j]->property["pathIDX"] = i;
			blendSubItems[i][j]->property["start"].setValue( resultItems[i][j]->property["graph"].value<Structure::Graph*>()->property["t"].toDouble() );
			blendSubItems[i][j]->property["end"].setValue( resultItems[i][j+1]->property["graph"].value<Structure::Graph*>()->property["t"].toDouble() );

			s->addItem( blendSubItems[i][j].data() );
		}
	}
}

void Blender::exportSelected()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	QString msg = "nothing selected.";
	
	foreach(QGraphicsItem * item, s->selectedItems())
	{
		BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
		if(renderItem)
		{
			Structure::Graph * g = renderItem->property["graph"].value<Structure::Graph*>();
			int idx = int(g->property["t"].toDouble() * 100);
			
			QString sname = g->property["sourceName"].toString();
			QString tname = g->property["sourceName"].toString();
			QString filename = sname + tname;

			// Create folder
			QDir d("dataset");	
			d.mkpath( filename );

			// Set it as current
			QDir::setCurrent( d.absolutePath() + "/" + filename );

			// Generate the geometry and export the structure graph
			s_manager->renderGraph(*g, filename + ".obj", false, 6, true);

			// Generate thumbnail
			// draw OpenGL and save to transparent .png here...

			// Restore
			QDir::setCurrent( d.absolutePath() + "/.." );
		}
	}

	emit( message("Exporting: " + msg) );

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());
}
