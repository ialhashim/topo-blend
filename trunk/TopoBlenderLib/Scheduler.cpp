#include "Scheduler.h"

Scheduler::Scheduler()
{

}

void Scheduler::drawBackground( QPainter * painter, const QRectF & rect )
{
	QGraphicsScene::drawBackground(painter,rect);

	// Draw tracks
	for(int i = 0; i < (int)items().size() * 1.25; i++)
	{
		int y = i * 17;
		painter->fillRect(-10, y, 4000, 16, QColor(80,80,80));
	}
}

void Scheduler::drawForeground( QPainter * painter, const QRectF & rect )
{
	int x = rect.x();
	int y = rect.y();

	// Draw ruler
	int rulerHeight = 25;
	int screenRight = x + rect.width();
	int screenBottom = y + rect.height();

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

		// Add to scene
		this->addItem( current );

		prev = current;
	}
}
