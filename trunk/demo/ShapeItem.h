#pragma once

#include "DemoGlobal.h"

class ShapeItem : public QGraphicsObject
{
public:
    explicit ShapeItem(QObject *parent = 0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    PropertyMap property;
    int width,height;

    int realHeight();
    int realWidth();

signals:
    
public slots:
    
};

Q_DECLARE_METATYPE(ShapeItem*)
