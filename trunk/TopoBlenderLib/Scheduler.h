#pragma once

#include <QGraphicsScene>
class Task;

class Scheduler : public QGraphicsScene
{
    Q_OBJECT

public:
    Scheduler();

	QVector<Task*> tasks;
	
	void schedule();
	void executeAll();

	void drawDebug();

protected:
	void drawBackground ( QPainter * painter, const QRectF & rect );
	void drawForeground ( QPainter * painter, const QRectF & rect );
};
