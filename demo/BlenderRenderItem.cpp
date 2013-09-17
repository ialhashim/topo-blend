#include <QPainter>
#include "BlenderRenderItem.h"

BlenderRenderItem::BlenderRenderItem(QPixmap pixmap) : pixmap(pixmap)
{
	setAcceptHoverEvents(true);

}

void BlenderRenderItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(option)
	Q_UNUSED(widget)

	painter->drawPixmap(pixmap.rect(), pixmap);

	if( isUnderMouse() )
	{
		painter->setBrush( Qt::NoBrush );
		painter->setPen( QPen(Qt::yellow, 2) );
		painter->drawRect( pixmap.rect() );
	}
}
