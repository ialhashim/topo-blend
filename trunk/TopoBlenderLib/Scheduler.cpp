#include <QApplication>
#include "Task.h"
#include "Scheduler.h"

Scheduler::Scheduler()
{
	rulerHeight = 25;
}

void Scheduler::drawBackground( QPainter * painter, const QRectF & rect )
{
	QGraphicsScene::drawBackground(painter,rect);

	int x = rect.x();
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
	int spacing = 100;
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

		// Minor tick
		for(int j = 1; j < minorTicks; j++)
		{
			double delta = double(spacing) / minorTicks;
			int minorX = curX + (j * delta);
			painter->drawLine(minorX, screenBottom, minorX, screenBottom - 5);
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

	foreach(Task * t, tasks)
	{
		// Create
		current = t;

		// Placement
		if(prev) current->moveBy(prev->x() + prev->width,prev->y() + (prev->height));
		current->currentTime = current->x();
		current->start = current->x();

		// Add to scene
		this->addItem( current );

		prev = current;
	}

	// Order and group here:


	// Prepare all tasks:
	foreach(Task * t, tasks)
	{
		t->prepare();
	}

	// Time-line slider
	slider = new TimelineSlider;
	slider->reset();
	this->connect( slider, SIGNAL(timeChanged(int)), SLOT(timeChanged(int)) );
    this->addItem( slider );
}

void Scheduler::executeAll()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	emit( progressStarted() );

	// Execute all tasks
	for(int i = 0; i < tasks.size(); i++)
	{
		Task * t = tasks[i];

		t->execute();

		// Collect resulting graphs
		allGraphs << t->outGraphs;

		// UI:
		int percent = (double(i) / (tasks.size() - 1)) * 100;
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
