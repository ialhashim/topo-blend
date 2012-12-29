#pragma once

#include <QGraphicsScene>
#include "Task.h"

class Scheduler : public QGraphicsScene
{
    Q_OBJECT

public:
    Scheduler( TopoBlender * topoBlender );

    TopoBlender * tp;

protected:
	void drawBackground ( QPainter * painter, const QRectF & rect );
	void drawForeground ( QPainter * painter, const QRectF & rect );
};
