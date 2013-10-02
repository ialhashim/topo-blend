#include "Blender.h"
#include "ProgressItem.h"
#include "BlendRenderItem.h"
#include "BlendPathRenderer.h"
#include "BlendPathSub.h"

#include "TopoBlender.h"
#include "Scheduler.h"
#include "SynthesisManager.h"
#include "ShapeRenderer.h"
#include "SchedulerWidget.h"

Blender::Blender(Scene * scene, QString title) : DemoPage(scene,title), m_gcorr(NULL), s_manager(NULL)
{
	this->isSample = true;
#ifdef QT_DEBUG
	this->isSample = false;
#endif

	this->graphItemWidth = s->width() * 0.15;
	
	this->numSuggestions = 4;
	this->numInBetweens = 5;

	setupBlendPathItems();

	// Progress bar
	progress = new ProgressItem("Working..", false, s);

	// Connections
    this->connect(this, SIGNAL(blendPathsReady()), SLOT(computeBlendPaths()));
    this->connect(this, SIGNAL(blendPathsDone()), SLOT(blenderDone()));
	this->connect(this, SIGNAL(becameHidden()), SLOT(cleanUp()));
	this->connect(this, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyReleased(QKeyEvent*)));
	this->connect(this, SIGNAL(blendDone()), SLOT(blenderAllResultsDone()));
}

void Blender::setupBlendPathItems()
{
	this->resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
	this->blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens) );

	// Create background items for each blend path
	int padding = 4;
	int blendPathHeight = (s->height() / (numSuggestions * 1.5)) * 0.99;
	int totalHeight = numSuggestions * (blendPathHeight + padding);

	int startY = (s->height() * 0.5 - totalHeight * 0.5) - 30;

	QColor color( 255, 180, 68, 100 );

	// Clear previously set items
	foreach(QGraphicsItemGroup * grp, blendPathsItems) s->removeItem(grp);
	blendPathsItems.clear();
	
	for(int i = 0; i < numSuggestions; i++)
	{
		QGraphicsRectItem * blendPathBack = new QGraphicsRectItem (0,0,s->width() - (2 * graphItemWidth), blendPathHeight);

		blendPathBack->setPen( Qt::NoPen );
		blendPathBack->setBrush( QColor (255, 255, 255, 10) );

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

	foreach(QGraphicsItem * item, auxItems) s->removeItem(item);
	auxItems.clear();

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

		QGraphicsItem * litem = s->addPath(lpath, QPen(color, 1));
		QGraphicsItem * ritem = s->addPath(rpath, QPen(color, 1));
		litem->setVisible(false);
		ritem->setVisible(false);

		auxItems.push_back(litem);
		auxItems.push_back(ritem);
	}

	itemHeight = blendPathHeight;

	// Add paths navigation buttons
	QRectF firstGroup = blendPathsItems.front()->sceneBoundingRect();
	QRectF lastGroup = blendPathsItems.back()->sceneBoundingRect();

	QGraphicsProxyWidget * prevButton = s->addButton(0,0, "prev", colorize( QImage(":/images/arrowUp.png"), QColor(255,153,0), 2 ) );
	QGraphicsProxyWidget * nextButton = s->addButton(0,0, "next", colorize( QImage(":/images/arrowDown.png"), QColor(255,153,0), 2 ) );

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
	renderer = new BlendPathRenderer(this, itemHeight);
	this->connect( renderer, SIGNAL(itemReady(QGraphicsItem*)), SLOT(blendResultDone(QGraphicsItem*)), Qt::DirectConnection );
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
		progress->setExtra("Preparing paths ");
		progress->show();

		qApp->setOverrideCursor(Qt::WaitCursor);
		qApp->processEvents();
		pathsTimer.start();
	}

	// Blending setup
	m_scheduler = QSharedPointer<Scheduler>( new Scheduler );
	m_blender = QSharedPointer<TopoBlender>( new TopoBlender( m_gcorr, m_scheduler.data() ) );
	
	srand(0);
	allSchedules = m_scheduler->manyRandomSchedules(100);
	resultsPage = 0;

	// Generate blend paths
	schedulePaths( m_scheduler, m_blender );

	// Synthesis requires a single instance of the blend process
	if( s_manager.isNull() )
	{
		int numSamples = 8000;

		s_manager = QSharedPointer<SynthesisManager>(new SynthesisManager(m_gcorr, m_scheduler.data(), m_blender.data(), numSamples));

		QVariant p_camera;
		p_camera.setValue( s->camera );
		s_manager->setProperty("camera", p_camera);

		// Progress connections
		progress->connect(s_manager.data(), SIGNAL(progressChanged(double)), SLOT(setProgress(double)));
		this->connect(s_manager.data(), SIGNAL(synthDataReady()), SLOT(synthDataReady()));
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

	emit( message(QString("Synthesis time [%1 ms]").arg(synthTimer.elapsed())) );
	emit( blendPathsReady() );
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
	//#pragma omp parallel for
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
		QVector<Structure::Graph*> inBetweens = curSchedule->interestingInBetweens( numInBetweens );

		for(int j = 0; j < numInBetweens; j++)
		{
			renderer->generateItem( inBetweens[j], i, j );
		}
	}
} 

void Blender::keyReleased( QKeyEvent* keyEvent )
{
	if(keyEvent->key() == Qt::Key_F)
	{
		this->isSample = !this->isSample;
		emit( message( QString("Sampling toggled to: %1").arg(isSample) ) );
		return;
	}

	if(keyEvent->key() == Qt::Key_M)
	{
		QRectF R;
		
		foreach(QGraphicsItemGroup * g, blendPathsItems) {g->hide(); R = R.united(g->sceneBoundingRect());}
		foreach(QGraphicsItem * item, auxItems)	item->hide();

		R.setSize(QSizeF(R.width() * 0.8, R.height() * 0.8));
		R.moveCenter(s->sceneRect().center());

		int columns = 20;
		int rows = numSuggestions;

		QVector< QVector<QPointF> > allPoints = QVector< QVector<QPointF> >(rows, QVector<QPointF>(columns));
		
		for(int y = 0; y < rows; y++){	
			for(int x = 0; x < columns; x++){
				double dx = double(x) / (columns-1);
				double dy = double(y) / (rows-1);
				int deltaY = (R.height() * 0.25) * pow((2 * abs(dx - 0.5)),3);
				QRectF r = R;
				r.adjust(0,deltaY,0,-deltaY);
				allPoints[y][x] = QPointF(	AlphaBlend(dx, r.topLeft().x(), r.topRight().x()),
											AlphaBlend(dy, r.topLeft().y(), r.bottomLeft().y()));
			}
		}

		QVector<QPainterPath> ellipPaths;

		foreach( QVector<QPointF> points, allPoints ){
			QPainterPath path;
			path.moveTo(points.at(0));
			for(int i = 1; i + 2 < points.size(); i++)
				path.cubicTo(points.at(i), points.at(i+1), points.at(i+2));
			ellipPaths.push_back(path);
		}

		// Re-arrange result items
		for(int i = 0; i < numSuggestions; i++){
			for(int j = 0; j < numInBetweens; j++){
				double t = double(j) / (numInBetweens - 1);

				int w = resultItems[i][j]->boundingRect().width() * 0.5;
				int h = resultItems[i][j]->boundingRect().height() * 0.5;

				QPointF p = ellipPaths[i].pointAtPercent(t);
				p.setX( (AlphaBlend(t, R.topLeft(), R.topRight())).x() );
				resultItems[i][j]->setPos(p.x() - w, p.y() - h);
			}
		}

		return;
	}

	if(keyEvent->key() == Qt::Key_Space)
	{
		foreach(QGraphicsItem * item, s->selectedItems())
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

	if(keyEvent->key() == Qt::Key_B)
	{
		SchedulerWidget * widget = new SchedulerWidget( m_scheduler.data() );
		widget->setAttribute(Qt::WA_DeleteOnClose);
		widget->show();
		return;
	}

	// Changing number of results
	{
		if(keyEvent->key() == Qt::Key_Equal) numSuggestions++;
		if(keyEvent->key() == Qt::Key_Minus) numSuggestions--;
		if(keyEvent->key() == Qt::Key_0) numInBetweens++;
		if(keyEvent->key() == Qt::Key_9) numInBetweens--;

		setupBlendPathItems();

		emit( message( QString("Paths [%1] / In-betweens [%2]").arg(numSuggestions).arg(numInBetweens) ) );
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

void Blender::blenderAllResultsDone()
{
	// Show UI elements
	foreach(QGraphicsItem * item, auxItems) item->setVisible(true);

	// Expand blend sequence near two items
	for(int i = 0; i < numSuggestions; i++)
	{
		QRectF pathRect = blendPathsItems[i]->boundingRect();
		double outterWidth = (pathRect.width() / numInBetweens);

		for(int j = 0; j + 1 < numInBetweens; j++)
		{
			BlendPathSubButton * subItemButton = new BlendPathSubButton(itemHeight * 0.5, itemHeight, this, 10);
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
	}

	emit( message("Exporting: " + msg) );

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());
}

void Blender::saveJob()
{
	foreach(QGraphicsItem * item, s->selectedItems())
	{
		BlendRenderItem * renderItem = qobject_cast<BlendRenderItem *>(item->toGraphicsObject());
		if(!renderItem) continue;
		
		int pathID = renderItem->property["pathID"].toInt();

		Structure::Graph * g = renderItem->property["graph"].value<Structure::Graph*>();
		QString sname = g->property["sourceName"].toString();
		QString tname = g->property["sourceName"].toString();
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

QWidget * Blender::viewer()
{
	return scene()->views().front();
}

void Blender::showPrevResults()
{
	if(!blendPaths.size()) return;

	resultsPage--;
	if(resultsPage < 0) 
	{
		resultsPage = 0;
		return;
	}

	// Clear generated items
	jobs.clear();
	resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
	blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens) );

	schedulePaths( m_scheduler, m_blender );

	// Generate more items
	emit( blendPathsReady() );
}

void Blender::showNextResults()
{
	if(!blendPaths.size()) return;

	resultsPage++;

	// Clear generated items
	jobs.clear();
	resultItems = QVector< QVector< QSharedPointer<BlendRenderItem> > >(numSuggestions, QVector< QSharedPointer<BlendRenderItem> >(numInBetweens) );
	blendSubItems = QVector< QVector< QSharedPointer<BlendPathSubButton> > >(numSuggestions, QVector< QSharedPointer<BlendPathSubButton> >(numInBetweens) );
	
	schedulePaths( m_scheduler, m_blender );

	// Generate more items
	emit( blendPathsReady() );
}
