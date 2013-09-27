#include "BlendPathSub.h"
#include "BlendPathRenderer.h"
#include "BlendRenderItem.h"
#include "Scheduler.h"

BlendPathSub::BlendPathSub(int x, int y, int height, int count, BlendPathSubButton *origin)
    : height(height), count(count), origin(origin)
{
	setAcceptHoverEvents(true);
	setZValue( origin->zValue() + 10 );

    int width = (count * height) + ((count-1) * 0.5 * height);
    m_geometry = QRectF(0,0,width,height);

	x -= m_geometry.width() * 0.5;
	y -= m_geometry.height() * 0.5;
	this->setPos(x,y);

	setup();

	viewer = origin->blender->viewer();
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(hideWhenInactive()));
	timer->start(300);
}

void BlendPathSub::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

	// Shadow
	QRectF r = boundingRect();
	r.adjust(4,4,-4,-4);
	painter->setPen(Qt::NoPen);
    painter->setBrush(QColor::fromRgbF(0, 0, 0, 0.4));
	painter->drawRoundedRect(r, 4, 4);

	// Background
	r.translate(QPointF(-4,-4));
	painter->setPen(QPen(QColor( 255, 180, 68, 50 ), 2));
	painter->setBrush(QColor(30,29,26,253).lighter());
	painter->drawRoundedRect(r, 4, 4);
}

void BlendPathSub::setup()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	int pathIDX = origin->property["pathIDX"].toInt();
	double start = origin->property["start"].toDouble();
	double end = origin->property["end"].toDouble();

	BlendPathRenderer * renderer = origin->blender->renderer;
	Scheduler * scheduler = origin->blender->blendPaths[pathIDX].scheduler.data();
	int N = scheduler->allGraphs.size();

	double step = (end - start) / (count-1);

	for(int i = 0; i < count; i++)
	{
		double t = start + (step * i);
		Structure::Graph * g = scheduler->allGraphs[qMin(N-1, int(t * N))];

		BlendRenderItem * item = renderer->genItem(g, -1, -1);
		item->pixmap = item->pixmap.copy(item->pixmap.rect().adjusted(0,0,0,-9));
		item->setZValue( zValue() + 10 );

		origin->blender->s->addItem( item );
		items.push_back( item );
		
		// Placement
		QRectF r = boundingRect();
		double outterWidth = (r.width() / count);
		int delta = 0.5 * (outterWidth  - item->boundingRect().width() );
		int x = scenePos().x() + (i * outterWidth) + delta;
		int y = scenePos().y();
		item->setPos(x, y);

		// Add sub button
		if (i+1 < count)
		{
			QRectF pathRect = mapRectToScene(boundingRect());
			double outterWidth = (pathRect.width() / count);
			int itemHeight = pathRect.height();

			BlendPathSubButton * button = new BlendPathSubButton(itemHeight * 0.5, itemHeight, origin->blender, zValue() + 10);
			button->setPos( QPointF(pathRect.x() + ((i+1) * outterWidth) - (button->boundingRect().width() * 0.5), pathRect.y()) );

			button->property["pathIDX"] = pathIDX;
			button->property["start"].setValue( t );
			button->property["end"].setValue( t + step );

			button->property["level"].setValue( origin->property["level"].toInt() + 1 );
			button->property["pos"] = origin->property["pos"];

			origin->blender->s->addItem( button );
			items.push_back( button );
		}
	}

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());
}

void BlendPathSub::hideWhenInactive()
{
	// Don't hide if an item is selected
	foreach(QGraphicsObject* item, items) if(item->isSelected()) return;

	// Don't hide if mouse cursor is on item
	QPoint p = viewer->mapFromGlobal(QCursor::pos());
	if(mapRectToScene(boundingRect()).contains(p)) return;

	// Clean up to hide
	foreach(QGraphicsObject* item, items) item->deleteLater();
	this->deleteLater();
}
