#include "BlendPathSub.h"

BlendPathSubButton::BlendPathSubButton(int width, int height, Blender * blender, int z_order) : blender(blender)
{
	setAcceptHoverEvents(true);
	setZValue(z_order);

    m_geometry = QRectF(0, 0, width, height);

	property["opacity"] = 0.05;
}

bool BlendPathSubButton::isOnTop()
{
	QRectF r = mapRectToScene(boundingRect().adjusted(10,10,-10,-10));
	QList<QGraphicsItem*> itemsAtRect = scene()->items(r, Qt::IntersectsItemBoundingRect, Qt::DescendingOrder);

	foreach(QGraphicsItem * i, itemsAtRect){
		if(i == this) return true;
		if(i->zValue() == zValue()) continue;
		break;
	}

	return false;
}

void BlendPathSubButton::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

	QRectF r = boundingRect();
	QPointF center = r.center();
	double radius = r.height() * 0.03;

	// DEBUG:
	//if( isOnTop() )	painter->drawRect( r );

	painter->setOpacity( property["opacity"].toDouble() );

	if( isUnderMouse() && isOnTop() )
		painter->setOpacity( 1.0 );

	// Shadow
	painter->setPen(QPen(QColor(0,0,0,50),2));
	painter->setBrush(QColor(0,0,0,128));
	painter->translate(2,2);
	painter->drawEllipse(center - QPointF(radius * 3,0), radius, radius);
	painter->drawEllipse(center, radius, radius);
	painter->drawEllipse(center + QPointF(radius * 3,0), radius, radius);

	// Big dots
	QColor color = QColor ( 255, 180, 68 );
	painter->setPen(Qt::NoPen);
	painter->setBrush( color );
	painter->translate(-2,-2);
	painter->drawEllipse(center - QPointF(radius * 3,0), radius, radius);
	painter->drawEllipse(center, radius, radius);
	painter->drawEllipse(center + QPointF(radius * 3,0), radius, radius);
}

void BlendPathSubButton::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	if( !isOnTop() ) return;

	Q_UNUSED(event)

	QPointF p = scenePos();
	QRectF r = boundingRect();

    int x = p.x() + (r.width() * 0.5);
    int y = p.y() + (r.height() * 0.5);

	int level = property["level"].toInt();
	if(level > 0)
	{
		QPointF position = this->property["pos"].toPointF();
		x = position.x() - 5;
		y = position.y() - 20;
	}
	this->property["pos"] = QPointF(x,y);
	
    BlendPathSub * subPath = new BlendPathSub(x, y, r.height(), 4, this);
    blender->s->addItem( subPath );
}
