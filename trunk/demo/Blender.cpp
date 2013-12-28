#include "Blender.h"
#include "Controls.h"
#include "ProgressItem.h"
#include "BlendRenderItem.h"
#include "BlendPathRenderer.h"
#include "BlendPathWidget.h"
#include "BlendPathSubButton.h"

#include "TopoBlender.h"
#include "Scheduler.h"
#include "Task.h"
#include "SynthesisManager.h"
#include "ShapeRenderer.h"
#include "SchedulerWidget.h"
#include "PathEvaluator.h"

typedef QVector< QSet<size_t> > ForcedGroups;
Q_DECLARE_METATYPE( ForcedGroups )
Q_DECLARE_METATYPE( ScheduleType )

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title), m_gcorr(NULL), s_manager(NULL), renderer(NULL), resultViewer(NULL)
{
	this->numSuggestions = 4;
	this->numInBetweens = 6;
	this->numSchedules = 300;
	this->isFiltering = false;
	this->isSample = true;

#ifdef QT_DEBUG
	this->isSample = false;
#endif

	this->graphItemWidth = s->width() * 0.15;

	setupBlendPathItems();

	// Progress bar
	progress = new ProgressItem("Working..", false, s);

	// Connections
	this->connect(this, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyReleased(QKeyEvent*)));

    this->connect(this, SIGNAL(blendPathsReady()), SLOT(computeBlendPaths()));
    this->connect(this, SIGNAL(blendPathsDone()), SLOT(blenderDone()));
	this->connect(this, SIGNAL(blendDone()), SLOT(blenderAllResultsDone()));

	// Paths evaluation
	pathsEval = new PathEvaluator(this);
	progress->connect(pathsEval, SIGNAL(progressChanged(double)), SLOT(setProgress(double)));
}

void Blender::setupBlendPathItems()
{
	clearResults();

	// Create background items for each blend path
	blendPathWidth = s->width() - (2 * graphItemWidth);
	blendPathHeight = ((s->height()-200) / numSuggestions);

	int padding = 3;
	int totalHeight = numSuggestions * (blendPathHeight + padding);

	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 25;

	QColor color( 255, 180, 68, 100 );

	// Clear previously set items
	foreach(QGraphicsProxyWidget * grp, blendPathsWidgets) s->removeItem(grp);
	blendPathsWidgets.clear();

	double w = blendPathWidth;
	double h = blendPathHeight;

	for(int i = 0; i < numSuggestions; i++)
	{
		double x = 0.5*s->width() - 0.5*w;
		double y = startY + (i * (blendPathHeight + padding));
		BlendPathWidget * widget = new BlendPathWidget(w,h);
		widget->setBackgroundColor( QColor (255, 255, 255, 5) );
		QGraphicsProxyWidget * witem = s->addWidget(widget);
		witem->setPos(x, y);
		witem->setVisible(false);
		witem->setAcceptHoverEvents(true);
		blendPathsWidgets.push_back(witem);
		widget->proxy = witem;
		items.push_back(witem);
	}

	// Show path indicators
	QRectF r0(0,0,graphItemWidth,graphItemWidth);	r0.moveTop((s->height() * 0.5) - (r0.height() * 0.5));
	QRectF r1 = r0; r1.moveRight(s->width());
	QPointF lstart = r0.center() + QPoint(r0.width() * 0.3,0);
	QPointF rstart = r1.center() - QPoint(r1.width() * 0.3,0);
	double tension = 0.5;

	foreach(QGraphicsItem * item, auxItems) s->removeItem(item);
	auxItems.clear();

	for(int i = 0; i < numSuggestions; i++)
	{
		QRectF r = blendPathsWidgets[i]->sceneBoundingRect();
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

		QGraphicsItem * litem = s->addPath(lpath, QPen(color, 1));
		QGraphicsItem * ritem = s->addPath(rpath, QPen(color, 1));
		litem->setVisible(false);
		ritem->setVisible(false);

		auxItems.push_back(litem);
		auxItems.push_back(ritem);
	}

	itemHeight = blendPathHeight;

	// Add paths navigation buttons
	QRectF firstGroup = blendPathsWidgets.front()->sceneBoundingRect();
	QRectF lastGroup = blendPathsWidgets.back()->sceneBoundingRect();

    prevButton = s->addButton(0,0, "prev", colorize( QImage(":/images/arrowUp.png"), QColor(255,153,0), 2 ) );
    nextButton = s->addButton(0,0, "next", colorize( QImage(":/images/arrowDown.png"), QColor(255,153,0), 2 ) );

	prevButton->setPos(firstGroup.right() - prevButton->boundingRect().width(), firstGroup.top() - padding - prevButton->boundingRect().height());
	nextButton->setPos(lastGroup.right() - nextButton->boundingRect().width(), lastGroup.bottom() + padding);
	prevButton->setVisible(false);
	nextButton->setVisible(false);

	auxItems.push_back(prevButton);
	auxItems.push_back(nextButton);

	// Connections
	this->connect(prevButton->widget(), SIGNAL(clicked()), SLOT(showPrevResults()));
	this->connect(nextButton->widget(), SIGNAL(clicked()), SLOT(showNextResults()));

	// Results renderer
	if( renderer ) delete renderer;
	renderer = new BlendPathRenderer(this, itemHeight);
	this->connect( renderer, SIGNAL(itemReady(QGraphicsItem*)), SLOT(blendResultDone(QGraphicsItem*)), Qt::DirectConnection );

	// Results viewer
	if( resultViewer ) delete resultViewer;
	resultViewer = new BlendPathRenderer(this, itemHeight * 2, true );
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

	cleanUp();
}

void Blender::setGraphCorresponder( GraphCorresponder * graphCorresponder )
{
	this->m_gcorr = graphCorresponder;
}

void Blender::schedulePaths( const QSharedPointer<Scheduler> & scheduler, const QSharedPointer<TopoBlender> & blender )
{
	blendPaths.clear();
	blendPaths.resize(numSuggestions);

	for(int i = 0; i < numSuggestions; i++)
	{
		// Shared properties
		BlendPath bp;
		bp.source = s->inputGraphs[0]->g;
		bp.target = s->inputGraphs[1]->g;
		bp.gcorr = this->m_gcorr;
		bp.blender = blender;

		// Per-path properties
		bp.scheduler = QSharedPointer<Scheduler>( scheduler->clone() );
		int idx = qMax(0, qMin(allSchedules.size() - 1, (resultsPage * numSuggestions) + i) );
		bp.scheduler->setSchedule( allSchedules[ idx ] );

		// Add blend path
		blendPaths[i] = bp;

		// Connections
		this->connect(bp.scheduler.data(), SIGNAL(progressChanged(int)), SLOT(pathProgressChanged()), Qt::DirectConnection);
	}
}

void Blender::preparePaths()
{    
	if(!s->isInputReady() || m_gcorr == NULL) return;
	emit( blendStarted() );

	isFinished = false;

	// UI and logging
	{	
		if( isFiltering ) 
			progress->setExtra("Filtering  ");
		else
			progress->setExtra("Preparing paths  ");

		progress->show();

		qApp->setOverrideCursor(Qt::WaitCursor);
		qApp->processEvents();
		pathsTimer.start();
	}

	// Blending setup
	m_scheduler = QSharedPointer<Scheduler>( new Scheduler );
	m_blender = QSharedPointer<TopoBlender>( new TopoBlender( m_gcorr, m_scheduler.data() ) );
	
	// Synthesis data preparation
	if( s_manager.isNull() )
	{
		int numSamples = 8000;

#ifdef QT_DEBUG
		numSamples = 100;
#endif

		s_manager = QSharedPointer<SynthesisManager>(new SynthesisManager(m_gcorr, m_scheduler.data(), m_blender.data(), numSamples));

		QVariant p_camera;
		p_camera.setValue( s->camera );
		s_manager->property["camera"] = p_camera;

		// Progress connections
		progress->connect(s_manager.data(), SIGNAL(progressChanged(double)), SLOT(setProgress(double)));
		this->connect(s_manager.data(), SIGNAL(synthDataReady()), SLOT(synthDataReady()));
	}

	// Apply any user-specified groups
	ForcedGroups forcedGroups = s->inputGraphs[0]->g->property["forceGroup"].value< ForcedGroups >();
	foreach(QSet<size_t> uids, forcedGroups)
	{
		// Retrieve all tasks of the set
		QVector<Task*> tasks;
		foreach(Structure::Node * n, m_scheduler->activeGraph->nodes){
			size_t uid = size_t(n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data());
			if(uids.contains(uid)) tasks.push_back( n->property["task"].value<Task*>() );
		}

		// Schedule them together
		int startTime = m_scheduler->totalExecutionTime();
		foreach(Task * t, tasks) startTime = qMin(startTime, t->start);
		foreach(Task * t, tasks) t->setStart( startTime );

		// Clean up
		m_scheduler->trimTasks();
	}

	/// Generate and sort blend paths:
	resultsPage = 0;
	srand(0);

	// Get 'k' schedules sorted by filter measure
	if( isFiltering )
		allSchedules = pathsEval->filteredSchedules( m_scheduler->manyRandomSchedules( numSchedules ) );
	else
		allSchedules = m_scheduler->manyRandomSchedules( numSchedules );

	schedulePaths( m_scheduler, m_blender );

	// UI and logging
	{
		qApp->restoreOverrideCursor();
		progress->startProgress();
		progress->setExtra("Generating samples  ");
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
	if( property["isOverrideSynth"].toBool() ) return;

	for(int i = 0; i < numSuggestions; i++)
		this->disconnect( blendPaths[i].scheduler.data() );

	emit( message(QString("Synthesis time [%1 ms]").arg(synthTimer.elapsed())) );
	emit( blendPathsReady() );
}

void executeJob( const QSharedPointer<Scheduler> & scheduler )
{
#ifdef QT_DEBUG
	scheduler->timeStep = 0.3;
#endif

	scheduler->executeAll();
}

void Blender::computeBlendPaths()
{
	progress->startProgress();
	progress->setExtra("Blend paths ");
	blendTimer.start();

	for(int i = 0; i < blendPaths.size(); i++) 
		jobs.push_back( blendPaths[i].scheduler );

	QtConcurrent::run(this, &Blender::computeBlendPathsThread);
	//computeBlendPathsThread();
}

void Blender::computeBlendPathsThread()
{
	#pragma omp parallel for
	for(int i = 0; i < blendPaths.size(); i++) 
	{
		executeJob( blendPaths[i].scheduler );
	}

	//QFuture<void> future = QtConcurrent::map(jobs, executeJob);
	//future.waitForFinished();

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

		progress->setExtra("Rendering ");
		progress->setProgress(0.0);
		renderTimer.start();
	}

	// For each blend path, render 'k' in-betweens
	for(int i = 0; i < numSuggestions; i++)
	{
		Scheduler * curSchedule = blendPaths[i].scheduler.data();
		ScheduleType schedule = curSchedule->getSchedule();


		QVector<Structure::Graph*> inBetweens = curSchedule->topoVaryingInBetweens( numInBetweens );

		for(int j = 0; j < numInBetweens; j++)
		{
			inBetweens[j]->moveCenterTo( AlphaBlend(inBetweens[j]->property["t"].toDouble(), 
													inBetweens[j]->property["sourceGraphCenter"].value<Vector3>(), 
													inBetweens[j]->property["targetGraphCenter"].value<Vector3>()), true);

			inBetweens[j]->property["schedule"].setValue( schedule );

			renderer->generateItem( inBetweens[j], i, j );
		}
	}
} 

void Blender::mousePress( QGraphicsSceneMouseEvent* mouseEvent )
{
	if(!visible) return;

	if( false )
	{
		// Block when controller is clicked
		foreach(QGraphicsItem * i, s->items(mouseEvent->scenePos())){
			QGraphicsProxyWidget * proxy = dynamic_cast<QGraphicsProxyWidget *>(i);
			if(!proxy) continue; else if(qobject_cast<Controls*>(proxy->widget())) return;
		}

		foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) 
		{
			if(proxy->sceneBoundingRect().contains(mouseEvent->scenePos())) continue;
			QGraphicsScene * s = ((BlendPathWidget*)proxy->widget())->scene;
			s->clearSelection();
		}
	}
}

void Blender::keyReleased( QKeyEvent* keyEvent )
{
	// Special keys across all pages:
	if( keyEvent->key() == Qt::Key_F )
	{
		this->isSample = !this->isSample;
		emit( message( QString("Sampling toggled to: %1").arg(isSample) ) );
		return;
	}

	if( keyEvent->key() == Qt::Key_E )
	{
		emit( showLogWindow() );

		emit( message( QString("Filtering [%1] Num schedules [%2]").arg(isFiltering).arg(numSchedules) ) );

		return;
	}

	// Regular keys
	if(!visible) return;

	// Re-draw results
	if( keyEvent->key() == Qt::Key_R ){
		showResultsPage();
		return;
	}

	// EXPERIMENTS:
	if( keyEvent->key() == Qt::Key_Q ){

		// Experiment: path evaluation
		//pathsEval->evaluatePaths();
		//pathsEval->clusterPaths();
		pathsEval->test_filtering();

		return;
	}
	if(keyEvent->key() == Qt::Key_W)
	{
		pathsEval->test_topoDistinct();
		return;
	}

	// Debug render graph function
	if(keyEvent->key() == Qt::Key_Backspace)
	{
		QList<QGraphicsItem*> allSelected;
		foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) allSelected << ((BlendPathWidget*)proxy->widget())->scene->selectedItems();

		foreach(QGraphicsItem * item, allSelected)
		{
			BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
			if(!renderItem) continue;

			QString filename = QString("testRender_%1.obj").arg(renderItem->property["pathID"].toInt());
			s_manager->renderGraph(*renderItem->graph(), filename, false, 6, true);
		}
	}

	// Show full schedule for selected item
	if(keyEvent->key() == Qt::Key_Space)
	{
		QList<QGraphicsItem*> allSelected;
		foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) allSelected << ((BlendPathWidget*)proxy->widget())->scene->selectedItems();

		foreach(QGraphicsItem * item, allSelected)
		{
			BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
			if(!renderItem) continue;

			int pathID = renderItem->property["pathID"].toInt();

			SchedulerWidget * widget = new SchedulerWidget( blendPaths[pathID].scheduler.data() );
			widget->setAttribute(Qt::WA_DeleteOnClose);
			widget->show();
		}

		return;
	}

	// Show initial schedule
	if(keyEvent->key() == Qt::Key_B)
	{
		SchedulerWidget * widget = new SchedulerWidget( m_scheduler.data() );
		widget->setAttribute(Qt::WA_DeleteOnClose);
		widget->show();
		return;
	}

	// Changing number of results
	{
		bool paramChanged = false;

		if(keyEvent->key() == Qt::Key_Equal) {numSuggestions++; paramChanged = true;}
		if(keyEvent->key() == Qt::Key_Minus) {numSuggestions--; paramChanged = true;}
		if(keyEvent->key() == Qt::Key_0) {numInBetweens++; paramChanged = true;}
		if(keyEvent->key() == Qt::Key_9) {numInBetweens--; paramChanged = true;}

		if(paramChanged){
			setupBlendPathItems();
			emit( message( QString("Paths [%1] / In-betweens [%2]").arg(numSuggestions).arg(numInBetweens) ) );
		}
	}
}

void Blender::blendResultDone(QGraphicsItem* done_item)
{
	BlendRenderItem * item = (BlendRenderItem*) done_item;

	int pathID = item->property["pathID"].toInt();
	int blendIDX = item->property["blendIDX"].toInt();

	resultItems[pathID][blendIDX] = QSharedPointer<BlendRenderItem>(item);

	//s->addItem( resultItems[pathID][blendIDX].data() );
	QGraphicsScene * scene = ((BlendPathWidget*)blendPathsWidgets[pathID]->widget())->scene;
	scene->addItem( resultItems[pathID][blendIDX].data() );

	this->connect(item, SIGNAL(doubleClicked(BlendRenderItem*)), SLOT(previewItem(BlendRenderItem*)));

	// Placement
	QRectF pathRect = blendPathsWidgets[pathID]->boundingRect();
	double outterWidth = pathRect.width() / numInBetweens;
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

		qApp->processEvents();

		if(numDone == N)
		{
            isFinished = true;

			progress->stopProgress();
			progress->hide();
			emit( message(QString("Render time [%1 ms]").arg(renderTimer.elapsed())) );

            emit( blendDone() );
		}
	}
}

void Blender::blenderAllResultsDone()
{
	// Show UI elements
	foreach(QGraphicsItem * item, auxItems) item->setVisible(true);

	// Expand blend sequence near two items
	for(int i = 0; i < numSuggestions; i++)
	{
		QRectF pathRect = blendPathsWidgets[i]->boundingRect();
		double outterWidth = (pathRect.width() / numInBetweens);

		double w = itemHeight * 0.5;
		double h = itemHeight;

		for(int j = 0; j < numInBetweens + 1; j++)
		{
			double x = pathRect.x() + ((j) * outterWidth) - (w * 0.5);

			if(j == 0) x = 0;
			if(j == numInBetweens) x = pathRect.width() - w;

			addBlendSubItem(x, pathRect.y(), w, h, i, j);
		}
	}

    prevButton->setEnabled(true);
    nextButton->setEnabled(true);

    emit( blendFinished() );
}

void Blender::addBlendSubItem(double x, double y, double w, double h, int i, int j)
{
	BlendPathSubButton * subItemButton = new BlendPathSubButton(w, h, this, 20);
	subItemButton->setPos( QPointF(x, y) );

	blendSubItems[i][j] = QSharedPointer<BlendPathSubButton>(subItemButton);
	blendSubItems[i][j]->property["pathIDX"] = i;

	double start = 0;
	double end = 1.0; 

	if(j > 0) start = resultItems[i][j-1]->graph()->property["t"].toDouble();
	if(j + 1 < numInBetweens) end = resultItems[i][j]->graph()->property["t"].toDouble();

	blendSubItems[i][j]->property["start"].setValue( start );
	blendSubItems[i][j]->property["end"].setValue( end );

	//s->addItem( blendSubItems[i][j].data() );
	QGraphicsScene * scene = ((BlendPathWidget*)blendPathsWidgets[i]->widget())->scene;
	scene->addItem( blendSubItems[i][j].data() );
}

QVector<BlendRenderItem *> Blender::selectedInBetween()
{
	QVector<BlendRenderItem *> result;

	QList<QGraphicsItem*> allSelected;
	foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) allSelected << ((BlendPathWidget*)proxy->widget())->scene->selectedItems();
	
	foreach(QGraphicsItem * item, allSelected)
	{
		BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
		if(!renderItem) continue;
		result.push_back( renderItem );
	}

	return result;
}

void Blender::clearSelectedInBetween()
{
	foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) 
	{
		QGraphicsScene * s = ((BlendPathWidget*)proxy->widget())->scene;
		s->clearSelection();
	}
}

void Blender::exportSelected()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	QString msg = "nothing selected.";
	
	foreach(BlendRenderItem * renderItem, selectedInBetween())
	{
		Structure::Graph * g = renderItem->graph();
		int idx = int(g->property["t"].toDouble() * 100);
			
		QString sname = g->property["sourceName"].toString();
		QString tname = g->property["targetName"].toString();
		QString filename = sname + tname + "." + QString::number(idx);

		// Create folder
		QDir d("dataset");	
		d.mkpath( filename );

		// Set it as current
		QDir::setCurrent( d.absolutePath() + "/" + filename );

		// Generate the geometry and export the structure graph
		s_manager->renderGraph(*g, filename, false, 5, true);

		// Generate thumbnail
		QString objFile = d.absolutePath() + "/" + filename + "/" + filename + ".obj";
		QString thumbnailFile = d.absolutePath() + "/" + filename + "/" + filename + ".png";
		ShapeRenderer::render( objFile ).save( thumbnailFile );

		// Send to gallery
		PropertyMap info;
		info["Name"] = filename;
		info["graphFile"] = d.absolutePath() + "/" + filename + "/" + filename + ".xml";
		info["thumbFile"] = d.absolutePath() + "/" + filename + "/" + filename + ".png";
		info["objFile"] = d.absolutePath() + "/" + filename + "/" + filename + ".obj";

		emit( exportShape(filename, info) );

		// Restore
		QDir::setCurrent( d.absolutePath() + "/.." );
	}

	emit( message("Exporting: " + msg) );

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());
}

void Blender::saveJob()
{
	QList<QGraphicsItem*> allSelected;
	foreach(QGraphicsProxyWidget * proxy, blendPathsWidgets) allSelected << ((BlendPathWidget*)proxy->widget())->scene->selectedItems();

	foreach(QGraphicsItem * item, allSelected)
	{
		BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
		if(!renderItem) continue;
		
		int pathID = renderItem->property["pathID"].toInt();

		if(pathID < 0 || pathID > blendPaths.size() - 1) continue; 

		Structure::Graph * g = renderItem->graph();
		QString sname = g->property["sourceName"].toString();
		QString tname = g->property["targetName"].toString();
		QString filename = sname + "." + tname;
		blendPaths[pathID].scheduler->saveSchedule(filename);

		QString sGraphName = m_gcorr->sgName();
		QString tGraphName = m_gcorr->tgName();
		QString graph_names = ( sGraphName + "_" + tGraphName ) + ".job";
		QString job_filename = QFileDialog::getSaveFileName(0, tr("Save Job"), graph_names, tr("Job Files (*.job)"));

		QFile job_file( job_filename );
		if (!job_file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
		QFileInfo jobFileInfo(job_file.fileName());
		QTextStream out(&job_file);

		// Create folders
		QDir jobDir( jobFileInfo.absolutePath() );

		QString sDir = jobDir.path() + "/Source/";
		QString tDir = jobDir.path() + "/Target/";
		jobDir.mkdir( sDir ); 
		jobDir.mkdir( tDir );

		// Save source and target graphs
		QString sRelative = "Source/" + sGraphName + ".xml";
		QString sgFileName = jobDir.path() + "/" + sRelative;
		m_blender->sg->saveToFile( sgFileName );

		QString tRelative = "Target/" + tGraphName + ".xml";
		QString tgFileName = jobDir.path() + "/"  + tRelative;
		m_blender->tg->saveToFile( tgFileName );

		// Save correspondence file
		QString correspondRelative = "correspondence.txt";
		QString correspondenceFileName = jobDir.path() + "/" + correspondRelative;
		m_gcorr->saveCorrespondences( correspondenceFileName, true );

		// Save the scheduler
		QString scheduleRelative = "schedule.txt";
		QString scheduleFileName = jobDir.path() + "/" + scheduleRelative;
		blendPaths[pathID].scheduler->saveSchedule( scheduleFileName );

		// Save paths & parameters
		out << sRelative << "\n";
		out << tRelative << "\n";
		out << correspondRelative << "\n";
		out << scheduleRelative << "\n";
		out << s_manager->samplesCount << "\t";
		out << DIST_RESOLUTION << "\t" << m_scheduler->timeStep << "\n";
		out << 7 << "\t" << numInBetweens << "\n";
		job_file.close();

		// Save samples
		//s_manager->saveSynthesisData( jobDir.path() + "/" );
	}
}

void Blender::clearResults()
{
	// Clear generated items
	jobs.clear();
	resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
	blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens + 2) );

	for(int i = 0; i < blendPathsWidgets.size(); i++)
	{
		BlendPathWidget * oldWidget = (BlendPathWidget*)blendPathsWidgets[i]->widget();
		oldWidget->buildScene(blendPathWidth, blendPathHeight);
	}
}

void Blender::cleanUp()
{
	// Clear results
	{
		clearResults();
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

QWidget * Blender::viewer()
{
	return scene()->views().front();
}

void Blender::showPrevResults()
{
    prevButton->setEnabled(false);
    emit( blendStarted() );

	if(!blendPaths.size()) return;

	resultsPage--;
	if(resultsPage < 0){
		resultsPage = 0;
		return;
	}
	showResultsPage();
}

void Blender::showNextResults()
{
    nextButton->setEnabled(false);
    emit( blendStarted() );

	if(!blendPaths.size()) return;
	resultsPage++;
	showResultsPage();
}

void Blender::showResultsPage()
{
	clearResults();

	schedulePaths( m_scheduler, m_blender );

	// Generate more items
	emit( blendPathsReady() );
}

void Blender::previewItem( BlendRenderItem* item )
{
	resultViewer->activeGraph = item->graph();

	// Placement
	QRectF viewerRect = resultViewer->geometry();
	viewerRect.moveCenter( QCursor::pos() );
	resultViewer->setGeometry( viewerRect.toRect() );

	resultViewer->show();
}

void Blender::emitMessage( QString msg )
{
	emit( message(msg) );
}

void Blender::filterStateChanged( int state )
{
	this->isFiltering = (state == Qt::Checked);

	emit( message( QString("Filter state changed: ").arg(isFiltering) ) );

	if( this->isFiltering ) numSchedules = 50;
}
