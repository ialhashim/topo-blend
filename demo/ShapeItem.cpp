#include "ShapeItem.h"

ShapeItem::ShapeItem(QObject *parent)
{
    Q_UNUSED(parent);
	
    width = 10;
    height = 10;
	
	setAcceptHoverEvents(true);
    //setFlags( QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable );
}

QRectF ShapeItem::boundingRect() const
{
    return QRectF(0, 0, width, height);
}

void ShapeItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
	
	//painter->drawRect(0, 0, width, height);
    painter->drawPixmap(0, 0, width, height, property["image"].value<QPixmap>());

	if( isUnderMouse() )
	{
		painter->setBrush( Qt::NoBrush );
		painter->setPen( QPen(QColor(255,255,0,128), 10) );
		painter->drawRect( boundingRect() );
	}
}

int ShapeItem::realHeight()
{
    return height * scale();
}

int ShapeItem::realWidth()
{
    return width * scale();
}

void ShapeItem::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	Q_UNUSED(event);
	emit( scrollToMe(this) );
}

void ShapeItem::hoverLeaveEvent( QGraphicsSceneHoverEvent * event )
{
	Q_UNUSED(event);
	update();
}
