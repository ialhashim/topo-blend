#include "BlendPathSub.h"

BlendPathSub::BlendPathSub(int width, int height, Blender * blender) : blender(blender)
{
	setAcceptHoverEvents(true);

    m_geometry = QRectF(0,0,width,height);

	property["opacity"] = 0.05;
}

void BlendPathSub::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

	QRectF r = boundingRect();
	QPointF center = r.center();
	double radius = r.height() * 0.03;

	// DEBUG:
	//painter->drawRect( r );

	if(!this->isUnderMouse())
		painter->setOpacity( property["opacity"].toDouble() );

	// Shadow
	painter->setPen(QPen(QColor(0,0,0,50),2));
	painter->setBrush(QColor(0,0,0,128));
	painter->translate(2,2);
	painter->drawEllipse(center - QPointF(radius * 3,0), radius, radius);
	painter->drawEllipse(center, radius, radius);
	painter->drawEllipse(center + QPointF(radius * 3,0), radius, radius);

	// Big dots
	QColor color = QColor ( 255, 180, 68 );
	painter->setPen(QPen( color, 1 ));
	painter->setBrush( color );
	painter->translate(-2,-2);
	painter->drawEllipse(center - QPointF(radius * 3,0), radius, radius);
	painter->drawEllipse(center, radius, radius);
	painter->drawEllipse(center + QPointF(radius * 3,0), radius, radius);
}

void BlendPathSub::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	int i = property["i"].toInt();
	int j = property["j"].toInt();

	QPointF p = scenePos();
	QPointF p_rect = blender->blendPathsItems[i]->scenePos();
	QRectF r = blender->blendPathsItems[i]->boundingRect();

	r.setWidth(r.width() * 0.5);

	blender->s->addRect(p.x() - (r.width() * 0.5), p.y() - 20, r.width(), r.height(), QPen(Qt::black,1), QColor( 255, 180, 68 ).darker());
}
