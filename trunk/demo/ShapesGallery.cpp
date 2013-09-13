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

ShapeItem *ShapesGallery::makeShapeItem( QString name, PropertyMap info )
{
    ShapeItem * item = new ShapeItem;

    item->property["name"] = name;
    item->property["graph"] = info["graphFile"];

    // Thumbnail
    QPixmap thumbnail( info["thumbFile"].toString() );
    item->property["image"].setValue(thumbnail);
    item->width = thumbnail.width();
    item->height = thumbnail.height();

    item->setScale(0.2);

    return item;
}

void ShapesGallery::loadDataset(DatasetMap dataset)
{
    foreach(QString shape, dataset.keys())
    {
        listA.push_back( makeShapeItem(shape, dataset[shape]) );
        listB.push_back( makeShapeItem(shape, dataset[shape]) );
    }

	if(!listA.size()) return;

    foreach(QGraphicsItem * item, listA) s->addItem(item);
    foreach(QGraphicsItem * item, listB) s->addItem(item);

    // Add selection boxes
    ShapeItem * item = (ShapeItem *)listA[0];
    int center = (s->height() * 0.5) - (item->realHeight() * 0.5);

    int penWidth = 5;
    QPen pen(Qt::yellow, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    items.push_back( s->addRect(  (penWidth*0.5) + 0, center, item->realWidth(), item->realHeight(), pen ) );
    items.push_back( s->addRect( -(penWidth*0.5) + s->width() - item->realWidth(), center, item->realWidth(), item->realHeight(), pen ) );

    // Tell scene about item size
    s->setProperty("itemWidth", item->realWidth());

    qDebug() << "Added [" << dataset.size() << "] shapes.";
}

void ShapesGallery::layout()
{
    arrangeList(listA, 0);
    arrangeList(listB, -1);

    indexA = 2;
    indexB = 3;

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

    bool isLeftSide = event->scenePos().x() < (s->width() * 0.5);
    bool isUp = event->delta() > 0;

    if(isLeftSide)
    {
        scrollTo(listA, isUp ? --indexA : ++indexA );
        emit( shapeChanged(0, listA[indexA]) );
    }
    else
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
