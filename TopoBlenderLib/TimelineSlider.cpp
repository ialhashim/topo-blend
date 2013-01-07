#include "TimelineSlider.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPainter>

TimelineSlider::TimelineSlider()
{
    icon = QImage(":/images/tick.png");
	setFlags(ItemIsMovable | ItemSendsGeometryChanges);

	isEnabled = false;
}

QRectF TimelineSlider::boundingRect() const
{
    return QRectF(icon.rect());
}

void TimelineSlider::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    painter->drawImage(0,0,icon);
}

QVariant TimelineSlider::itemChange( GraphicsItemChange change, const QVariant & value )
{
    if (scene() && change == ItemPositionChange)
    {
        QPointF newPos = value.toPointF();

		if(!isEnabled) 
		{
			newPos.setX(this->x());
			newPos.setY(myY);
		}

        newPos.setX(qMax( -0.5 * icon.width(), newPos.x()));
        newPos.setY(myY);

        return newPos;
    }

	if(isEnabled) emit ( timeChanged(currentTime()) );

    return QGraphicsItem::itemChange(change, value);
}

void TimelineSlider::forceY( int newY )
{
	myY = newY;
	setPos(pos().x(), pos().y());
}

void TimelineSlider::reset()
{
	setX(-0.5 * icon.width());
	setY(0);
}

int TimelineSlider::currentTime()
{
	return this->pos().x() + (0.5 * icon.width());
}

void TimelineSlider::enable()
{
	isEnabled = true;
}
