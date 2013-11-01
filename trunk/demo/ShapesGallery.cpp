#include <QGraphicsSceneWheelEvent>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include "ShapesGallery.h"

ShapesGallery::ShapesGallery(Scene * scene, QString title) : DemoPage(scene,title)
{
    show();
}

void ShapesGallery::hide()
{
    QVector<QGraphicsItem *> all;
    all << listA << listB;
    foreach(QGraphicsItem * item, all) item->hide();

    DemoPage::hide();
}

void ShapesGallery::show()
{
    QVector<QGraphicsItem *> all;
    all << listA << listB;
    foreach(QGraphicsItem * item, all) item->show();

    DemoPage::show();

    if(!s->isInputReady()) return;

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;
    animGroup->addAnimation( s->inputGraphs[0]->animateTo( s->graphRect(0) ) );
    animGroup->addAnimation( s->inputGraphs[1]->animateTo( s->graphRect(1) ) );
    animGroup->start( QAbstractAnimation::DeleteWhenStopped );
}

ShapeItem *ShapesGallery::makeShapeItem( QString name, PropertyMap info, int idx, bool isRight )
{
    ShapeItem * item = new ShapeItem;

    item->property["name"] = name;
    item->property["graph"] = info["graphFile"];

    // Thumbnail
	QPixmap thumbnail( info["thumbFile"].toString() );
	item->property["image"].setValue(thumbnail);
    item->width = thumbnail.width();
    item->height = thumbnail.height();

	item->property["idx"] = idx;
	item->property["isRight"] = isRight;
	this->connect(item, SIGNAL(scrollToMe(ShapeItem*)), SLOT(scrollToItem(ShapeItem*)));

	double s = 128.0 / item->width;

    item->setScale( s );

    return item;
}

void ShapesGallery::loadDataset(DatasetMap dataset)
{
    foreach(QString shape, dataset.keys())
    {
        listA.push_back( makeShapeItem(shape, dataset[shape], listA.size(), false) );
        listB.push_back( makeShapeItem(shape, dataset[shape], listB.size(), true) );
    }

	if(!listA.size()) return;

    foreach(QGraphicsItem * item, listA) s->addItem(item);
    foreach(QGraphicsItem * item, listB) s->addItem(item);

	// Default item
	ShapeItem * item = (ShapeItem *)listA[0];

    // Add selection boxes
	{
		int center = (s->height() * 0.5) - (item->realHeight() * 0.5);
		int penWidth = 3;
		QPen pen(Qt::yellow, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
		QGraphicsItem * r1 = s->addRect(  (penWidth*0.5) + 0, center, item->realWidth(), item->realHeight(), pen );
		QGraphicsItem * r2 = s->addRect( -(penWidth*0.5) + s->width() - item->realWidth(), center, item->realWidth(), item->realHeight(), pen );

		QGraphicsDropShadowEffect * shadow = new QGraphicsDropShadowEffect;
		shadow->setColor( QColor(0,0,0,200) );
		shadow->setOffset(2);
		shadow->setBlurRadius(4);

		r1->setGraphicsEffect(shadow);
		r2->setGraphicsEffect(shadow);

		items.push_back( r1 );
		items.push_back( r2 );
	}

    // Tell scene about item size
    s->setProperty("itemWidth", item->realWidth());

	QString log = QString("Added [%1] shapes.").arg(listA.size());
	emit (message(log));
}

void ShapesGallery::appendShape(QString name, PropertyMap data)
{
	ShapeItem * prevA = (ShapeItem *) listA.back();
	ShapeItem * prevB = (ShapeItem *) listB.back();

	listA.push_back( makeShapeItem(name, data, listA.size(), false) );
	listB.push_back( makeShapeItem(name, data, listB.size(), true) );

	QGraphicsItem * itemA = listA.back();
	QGraphicsItem * itemB = listB.back();

	// Add to scene
	s->addItem( itemA );
	s->addItem( itemB );

	// Placement
	itemA->setPos(prevA->scenePos());
	itemA->moveBy(0, prevA->realHeight());

	itemB->setPos(prevB->scenePos());
	itemB->moveBy(0, prevB->realHeight());

	itemA->setVisible( false );
	itemB->setVisible( false );
}

int ShapesGallery::indexOf( QString graphName )
{
	int foundIDX = -1;

	for(int i = 0; i < listA.size(); i++){
		ShapeItem * item = (ShapeItem *)listA[i];
		if(item->property["name"] == graphName){
			foundIDX = i;
			break;
		}
	}

	return foundIDX;
}

void ShapesGallery::layout()
{
    arrangeList(listA, 0);
    arrangeList(listB, -1);

    indexA = qMax(0, indexOf("ChairBasic1"));
    indexB = qMax(0, indexOf("ChairBasic2"));

	if(!listA.size()) return;

    scrollTo(listA, indexA);
    emit( shapeChanged(0, listA[indexA]) );

    scrollTo(listB, indexB);
    emit( shapeChanged(1, listB[indexB]) );
}

void ShapesGallery::arrangeList( QVector<QGraphicsItem*> & list, int x )
{
    for(int i = 0; i < list.size(); i++)
    {
        ShapeItem * item = (ShapeItem *)list[i];
        if(x < 0) x = s->width() - item->realWidth();
        item->setPos(x, i * item->realHeight());
    }
}

void ShapesGallery::wheelEvent(QGraphicsSceneWheelEvent * event)
{
    if(!this->isVisible()) return;

    bool isLeftSide = event->scenePos().x() < (s->width() * 0.6);
	bool isRightSide = event->scenePos().x() > (s->width() * 0.4);
    bool isUp = event->delta() > 0;

    if(isLeftSide)
    {
        scrollTo(listA, isUp ? --indexA : ++indexA );
        emit( shapeChanged(0, listA[indexA]) );
    }
    
	if(isRightSide)
    {
        scrollTo(listB, isUp ? --indexB : ++indexB );
        emit( shapeChanged(1, listB[indexB]) );
    }
}

void ShapesGallery::scrollTo( QVector<QGraphicsItem*> & list, int & index )
{
	if(!list.size()) return;

    // bound check
    index = qMax(qMin(index,list.size()-1), 0);

    ShapeItem * selected = (ShapeItem *)list[index];
    int itemHeight = selected->realHeight();
    int center = (s->height() * 0.5) - (itemHeight * 0.5);
    int delta = center - list[index]->y();

    QParallelAnimationGroup * animGroup = new QParallelAnimationGroup;

    for(int i = 0; i < list.size(); i++)
    {
        ShapeItem * item = (ShapeItem *)list[i];

        QPropertyAnimation* anim = new QPropertyAnimation;
        anim->setTargetObject(item);
        anim->setPropertyName("y");
        anim->setDuration(200);
        anim->setEndValue(item->y() + delta);
        anim->setEasingCurve(QEasingCurve::OutQuad);
        animGroup->addAnimation(anim);
    }

    animGroup->start( QAbstractAnimation::DeleteWhenStopped );
}

void ShapesGallery::scrollToItem(ShapeItem* item)
{
	bool isRight = item->property["isRight"].toBool();

	int * idx = isRight ? &indexB : &indexA;
	QVector<QGraphicsItem*> * list = isRight ? &listB : &listA;

	// Set index to item selected
	*idx = item->property["idx"].toInt();

	scrollTo(*list, *idx);
	emit( shapeChanged(isRight, list->at(*idx)) );
}
