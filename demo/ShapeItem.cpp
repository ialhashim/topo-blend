#include "ShapeItem.h"

ShapeItem::ShapeItem(QObject *parent)
{
    Q_UNUSED(parent);

    width = 10;
    height = 10;

    //setFlags( ItemIsMovable | ItemIsSelectable );
}

QRectF ShapeItem::boundingRect() const
{
    return QRectF(0, 0, width, height);
}

void ShapeItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

    painter->drawPixmap(0, 0, width, height, property["image"].value<QPixmap>());
    //painter->drawRect(0, 0, width, height);
}

int ShapeItem::realHeight()
{
    return height * scale();
}

int ShapeItem::realWidth()
{
    return width * scale();
}
