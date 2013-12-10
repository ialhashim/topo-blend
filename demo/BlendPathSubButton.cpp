#include <QGraphicsScene>
#include <QGraphicsView>

#include "BlendPathSubButton.h"
#include "BlendPathWidget.h"
#include "BlendRenderItem.h"
#include "BlendPathRenderer.h"

BlendPathSubButton::BlendPathSubButton(int width, int height, Blender * blender, int z_order) : blender(blender)
{
	setAcceptHoverEvents(true);
	setZValue(z_order);

    m_geometry = QRectF(0, 0, width, height);

	property["opacity"] = 0.1;
}

void BlendPathSubButton::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

	QRectF r = boundingRect();
	QPointF center = r.center();
	double radius = r.height() * 0.025;

	// DEBUG:
	//painter->drawRect( r );

	painter->setOpacity( property["opacity"].toDouble() );

	if( isUnderMouse() )
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

bool BlendPathSubButton::isUnderMouse()
{
	double h = boundingRect().height();
	BlendPathWidget * w = (BlendPathWidget *)scene()->views().front()->parentWidget();
	QPointF mouseInsideMainScene = w->proxy->scene()->views().front()->mapFromGlobal(QCursor::pos());
	QRectF rectInsideMainScene = sceneBoundingRect().translated(w->proxy->pos()).adjusted(0,h*0.25,0,-h*0.25);
	QPointF p = scene()->views().front()->mapToScene(0,0);
	return rectInsideMainScene.contains(mouseInsideMainScene + p);
}

void BlendPathSubButton::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	Q_UNUSED(event);
	if(!isUnderMouse()) return;

	qApp->setOverrideCursor(Qt::WaitCursor);

	int pathIDX = property["pathIDX"].toInt();
	double start = property["start"].toDouble();
	double end = property["end"].toDouble();

	BlendPathRenderer * renderer = blender->renderer;
	Scheduler * scheduler = blender->blendPaths[pathIDX].scheduler.data();
	int N = scheduler->allGraphs.size();

	int count = 2;

	double step = (end - start) / (count+1);

	int iwidth = blender->itemHeight * 0.8;
	QRectF r (0, 0, (iwidth*count) + (count * iwidth*0.5), blender->itemHeight);
	r.moveCenter(sceneBoundingRect().center());

	// 1) Hide the button
	this->setVisible( false );

	// 2) Locate and move items around new ones
	QPainterPath path;
	path.addRect(scene()->sceneRect());
	
	// Setup movement
	QSequentialAnimationGroup * bothAnim = new QSequentialAnimationGroup;
	QParallelAnimationGroup * prevItemsGroup = new QParallelAnimationGroup;

	foreach(QGraphicsItem* item, scene()->items()){
		QGraphicsObject * obj = item->toGraphicsObject();
		if(!obj) continue;
		QPropertyAnimation * anim = new QPropertyAnimation(obj, "pos");
		anim->setDuration( 300 );
		anim->setStartValue( obj->pos() );

		double direction = (obj->x() >= this->sceneBoundingRect().center().x() - this->boundingRect().width() ? 1 : -1);

		anim->setEndValue( obj->pos() + (direction * QPointF(r.width() * 0.5,0)) );
		anim->setEasingCurve( QEasingCurve::InOutQuad );
		prevItemsGroup->addAnimation( anim );
	}

	// 3) Generate and animate the new items
	QParallelAnimationGroup * newItemsGroup = new QParallelAnimationGroup;
	QVector<QGraphicsObject*> newItems;

	for(int i = 0; i < count; i++)
	{
		double t = start + (step * (i+1));
		Structure::Graph * g = scheduler->allGraphs[qMin(N-1, int(t * N))];

		BlendRenderItem * item = renderer->genItem(g, -1, -1);
		item->pixmap = item->pixmap.copy(item->pixmap.rect());

		scene()->addItem( item );
		newItems.push_back( item );

		blender->connect(item, SIGNAL(doubleClicked(BlendRenderItem*)), SLOT(previewItem(BlendRenderItem*)));

		// Placement
		double outterWidth = (r.width() / count);
		int delta = 0.5 * (outterWidth  - item->boundingRect().width() );
		int x = r.x() + (i * outterWidth) + delta;
		int y = r.y();
		item->setPos(x, y);
		item->setOpacity( 0.0 );

		// Add sub buttons
		{
			QRectF pathRect = mapRectToScene(boundingRect());
			int itemHeight = pathRect.height();

			BlendPathSubButton * button = new BlendPathSubButton(itemHeight * 0.5, itemHeight, blender, zValue() + 10);
			button->setPos( QPointF(item->x() + iwidth, pathRect.y()) );
			button->setOpacity( 0.0 );
			button->property["pathIDX"] = pathIDX;
			button->property["start"].setValue( t );
			button->property["end"].setValue( t + step );
			button->property["level"].setValue( property["level"].toInt() + 1 );
			button->property["pos"] = property["pos"];

			scene()->addItem( button );
			newItems.push_back( button );

			// Special first button
			if(i == 0)
			{
				BlendPathSubButton * button = new BlendPathSubButton(itemHeight * 0.5, itemHeight, blender, zValue() + 10);
				button->setPos( QPointF(item->x() - (iwidth * 0.5), pathRect.y()) );
				button->setOpacity( 0.0 );
				button->property["pathIDX"] = pathIDX;
				button->property["start"].setValue( start );
				button->property["end"].setValue( t );
				button->property["level"].setValue( property["level"].toInt() + 1 );
				button->property["pos"] = property["pos"];
				scene()->addItem( button );
				newItems.push_back( button );
			}
		}
	}

	foreach(QGraphicsObject * obj, newItems){
		QPropertyAnimation * anim = new QPropertyAnimation(obj, "opacity");
		anim->setDuration( 300 );
		anim->setStartValue( 0.0 );
		anim->setEndValue( 1.0 );
		anim->setEasingCurve( QEasingCurve::InCubic );
		newItemsGroup->addAnimation( anim );
	}

	// Execute animation
	bothAnim->addAnimation( prevItemsGroup );
	bothAnim->addAnimation( newItemsGroup );
	bothAnim->start( QAbstractAnimation::DeleteWhenStopped );

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());
}
