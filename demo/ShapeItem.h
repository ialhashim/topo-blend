#pragma once

#include "DemoGlobal.h"

class ShapeItem : public QGraphicsObject
{
	Q_OBJECT
public:
    explicit ShapeItem(QObject *parent = 0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    PropertyMap property;
    int width,height;

    int realHeight();
    int realWidth();

protected:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent * event);
	virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent * event);

signals:
    void scrollToMe(ShapeItem*);

public slots:
    
};

Q_DECLARE_METATYPE(ShapeItem*)
