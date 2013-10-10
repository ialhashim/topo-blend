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

	BlendPathWidget * w = (BlendPathWidget *)scene()->views().front()->parentWidget();
	QPointF mouseInsideMainScene = w->proxy->scene()->views().front()->mapFromGlobal(QCursor::pos());

	double pad = 20;
	QRectF rectInsideMainScene = sceneBoundingRect().adjusted(pad,0,-pad,0).translated(w->proxy->pos());
	QPointF p = scene()->views().front()->mapToScene(0,0);

	if( isSelected() || rectInsideMainScene.contains(mouseInsideMainScene + p) )
	{
		painter->setBrush( Qt::NoBrush );
		painter->setPen( QPen(Qt::yellow, 2) );
		painter->drawRect( pixmap.rect() );
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
