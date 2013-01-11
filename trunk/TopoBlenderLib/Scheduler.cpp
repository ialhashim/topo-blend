#include <QApplication>

#include "Task.h"
#include "Scheduler.h"

#include "Synthesizer.h"

Scheduler::Scheduler()
{
	rulerHeight = 25;
	sourceGraph = targetGraph = NULL;
}

void Scheduler::drawBackground( QPainter * painter, const QRectF & rect )
{
	QGraphicsScene::drawBackground(painter,rect);

	int y = rect.y();
	int screenBottom = y + rect.height();

	// Draw tracks
	for(int i = 0; i < (int)items().size() * 1.25; i++)
	{
		int y = i * 17;
		painter->fillRect(-10, y, 4000, 16, QColor(80,80,80));
	}

	// Draw current time indicator
	int ctime = slider->currentTime();
	painter->fillRect(ctime, 0, 1, screenBottom, QColor(0,0,0,128));
}

void Scheduler::drawForeground( QPainter * painter, const QRectF & rect )
{
	int x = rect.x();
	int y = rect.y();

	// Draw ruler
	int screenBottom = y + rect.height();
	painter->fillRect(x, screenBottom - rulerHeight, rect.width(), rulerHeight, QColor(64,64,64));

	// Draw yellow line
	int yellowLineHeight = 2;
	painter->fillRect(x, screenBottom - rulerHeight - yellowLineHeight, rect.width(), yellowLineHeight, Qt::yellow);

	// Draw text & ticks
	int totalTime = totalExecutionTime();
	int spacing = totalTime / 10;
	int timeEnd = 10;
	int minorTicks = 5;
	painter->setPen(Qt::gray);
	QFontMetrics fm(painter->font());

	for(int i = 0; i <= timeEnd; i++)
	{
		double time = double(i) / timeEnd;

		int curX = i * spacing;

		QString tickText = QString("00:%1").arg(time);
		painter->drawText(curX - (fm.width(tickText) * 0.5), screenBottom - 14, tickText );

		// Major tick
		painter->drawLine(curX, screenBottom, curX, screenBottom - 10);

		if(i != timeEnd)
		{
			// Minor tick
			for(int j = 1; j < minorTicks; j++)
			{
				double delta = double(spacing) / minorTicks;
				int minorX = curX + (j * delta);
				painter->drawLine(minorX, screenBottom, minorX, screenBottom - 5);
			}
		}
	}

	slider->forceY(screenBottom - rulerHeight - 10);
	slider->setY(slider->myY);
	painter->translate(slider->pos());
	slider->paint(painter, 0, 0);
}

void Scheduler::schedule()
{
	Task *current, *prev = NULL;

	foreach(Task * task, tasks)
	{
		// Create
		current = task;

		// Placement
		if(prev) current->moveBy(prev->x() + prev->width,prev->y() + (prev->height));
		current->currentTime = current->x();
		current->start = current->x();

		// Add to scene
		this->addItem( current );

		prev = current;
	}

	// Order and group here:
	this->order();

	// Prepare all tasks:
	foreach(Task * task, tasks)
	{
		task->prepare();
	}

	// Time-line slider
	slider = new TimelineSlider;
	slider->reset();
	this->connect( slider, SIGNAL(timeChanged(int)), SLOT(timeChanged(int)) );
    this->addItem( slider );
}

void Scheduler::order()
{
	QMultiMap<Task::TaskType,Task*> tasksByType;

	foreach(Task * task, tasks)
		tasksByType.insert(task->type, task);

	int curStart = 0;

	for(int i = Task::SHRINK; i <= Task::GROW; i++)
	{
		QList<Task*> curTasks = tasksByType.values(Task::TaskType(i));
		if(!curTasks.size()) continue;

		int futureStart = curStart;

		foreach(Task* t, curTasks){
			t->setStart(curStart);
			futureStart = qMax(futureStart, t->endTime());
		}

		curStart = futureStart;
	}
}

void Scheduler::executeAll()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	emit( progressStarted() );

	double timeStep = 0.001;
	int totalTime = totalExecutionTime();
	isForceStop = false;

	QVector<Task*> allTasks = tasksSortedByStart();

	// Execute all tasks
	for(double globalTime = 0; globalTime <= (1.0 + timeStep); globalTime += timeStep)
	{
		QElapsedTimer timer; timer.start();

		for(int i = 0; i < (int)allTasks.size(); i++)
		{
			Task * task = allTasks[i];
			double localTime = task->localT( globalTime * totalTime );
			task->execute( localTime );
		}

		// Output current active graph:
		allGraphs.push_back( new Structure::Graph( *(tasks.front()->active) ) );

		if(isForceStop) break;

		// UI - visual indicator:
		int percent = globalTime * 100;
		emit( progressChanged(percent) );
	}

	slider->enable();

	emit( progressDone() );

	qApp->restoreOverrideCursor();
}

void Scheduler::drawDebug()
{
	foreach(Task * t, tasks)
		t->drawDebug();
}

int Scheduler::totalExecutionTime()
{
	int endTime = 0;

	foreach(Task * t, tasks)
		endTime = qMax(endTime, t->endTime());

	return endTime;
}

void Scheduler::timeChanged( int newTime )
{
	int idx = allGraphs.size() * (double(newTime) / totalExecutionTime());

	idx = qRanged(0, idx, allGraphs.size() - 1);

	emit( activeGraphChanged(allGraphs[idx]) );
}

void Scheduler::doBlend()
{
	emit( startBlend() );
}

QVector<Task*> Scheduler::tasksSortedByStart()
{
	QMap<Task*,int> tasksMap;
	typedef QPair<int, Task*> IntTaskPair;
	foreach(Task* t, tasks) tasksMap[t] = t->start;
	QList< IntTaskPair > sortedTasksList = sortQMapByValue<Task*,int>( tasksMap );
	QVector< Task* > sortedTasks; 
	foreach( IntTaskPair p, sortedTasksList ) sortedTasks.push_back(p.second);
	return sortedTasks;
}

void Scheduler::stopExecution()
{
	isForceStop = true;
}

void Scheduler::startAllSameTime()
{
	foreach(Task * t, tasks)
		t->setX(0);
}

void Scheduler::prepareSynthesis()
{
	foreach(Task * t, tasks)
	{
		
	}
}
