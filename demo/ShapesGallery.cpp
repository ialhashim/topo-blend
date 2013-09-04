#include <QGraphicsSceneWheelEvent>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include "ShapesGallery.h"

ShapesGallery::ShapesGallery(Scene * scene) : s(scene)
{
    QFont titleFont("", 30);
    QGraphicsTextItem * title = s->addText("Select two shapes", titleFont);
    title->setPos( (s->width() * 0.5) - (title->boundingRect().width() * 0.5),
                   s->height() * 0.1);
    title->setDefaultTextColor(Qt::white);
    items.push_back(title);

    show();
}

bool ShapesGallery::isVisible()
{
    return visible;
}

void ShapesGallery::hide()
{
    this->visible = false;

    // Hide items and shapes
    QVector<QGraphicsItem *> all;
    all << items << listA << listB;
    foreach(QGraphicsItem * item, all) item->hide();
}

void ShapesGallery::show()
{
    this->visible = true;

    // Show items and shapes
    QVector<QGraphicsItem *> all;
    all << items << listA << listB;
    foreach(QGraphicsItem * item, all) item->show();
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

    item->setScale(0.4);

    return item;
}

void ShapesGallery::loadDataset(DatasetMap dataset)
{
    foreach(QString shape, dataset.keys())
    {
        listA.push_back( makeShapeItem(shape, dataset[shape]) );
        listB.push_back( makeShapeItem(shape, dataset[shape]) );
    }

    foreach(QGraphicsItem * item, listA) s->addItem(item);
    foreach(QGraphicsItem * item, listB) s->addItem(item);

    // Add selection boxes
    ShapeItem * item = (ShapeItem *)listA[0];
    int center = (s->height() * 0.5) - (item->realHeight() * 0.5);

    int penWidth = 5;
    QPen pen(Qt::yellow, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    items.push_back( s->addRect(  (penWidth*0.5) + 0, center, item->realWidth(), item->realHeight(), pen ) );
    items.push_back( s->addRect( -(penWidth*0.5) + s->width() - item->realWidth(), center, item->realWidth(), item->realHeight(), pen ) );

    qDebug() << "Added [" << dataset.size() << "] shapes.";
}

void ShapesGallery::layout()
{
    arrangeList(listA, 0);
    arrangeList(listB, -1);

    indexA = 0;
    indexB = 1;

    scrollTo(listA, indexA);
    scrollTo(listB, indexB);
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
