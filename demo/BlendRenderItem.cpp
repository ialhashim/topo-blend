#include <QDebug>
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsWidget>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsProxyWidget>
#include "BlendRenderItem.h"
#include "BlendPathWidget.h"

BlendRenderItem::BlendRenderItem(QPixmap pixmap) : pixmap(pixmap)
{
	setAcceptHoverEvents( true );
	setZValue( 10 );
	setFlags( QGraphicsItem::ItemIsSelectable );
}

void BlendRenderItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	Q_UNUSED(option)
	Q_UNUSED(widget)

	painter->drawPixmap(pixmap.rect(), pixmap);

	if( isSelected() || isUnderMouse() )
	{
		painter->setBrush( Qt::NoBrush );
		painter->setPen( QPen(Qt::yellow, 2) );
		painter->drawRect( pixmap.rect().adjusted(10,10,-10,-10) );
	}

	/// DEBUG:
	//painter->drawText(10,20, QString::number(graph()->property["t"].toDouble()) );
}

void BlendRenderItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
	emit( doubleClicked(this) );
	QGraphicsObject::mouseDoubleClickEvent(event);
}

Structure::Graph * BlendRenderItem::graph()
{
	return property["graph"].value<Structure::Graph*>();
}

bool BlendRenderItem::isUnderMouse()
{
	double width = boundingRect().width();
	double pad = width * 0.25;

	BlendPathWidget * w = (BlendPathWidget *)scene()->views().front()->parentWidget();
	QPointF mouseInsideMainScene = w->proxy->scene()->views().front()->mapFromGlobal(QCursor::pos());
	QRectF rectInsideMainScene = sceneBoundingRect().adjusted(pad,0,-pad,0).translated(w->proxy->pos());
	QPointF p = scene()->views().front()->mapToScene(0,0);
	return rectInsideMainScene.contains(mouseInsideMainScene + p);
}
