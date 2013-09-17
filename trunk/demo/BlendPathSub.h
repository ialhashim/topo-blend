#pragma once

#include <QGraphicsObject>
#include "Blender.h"

class BlendPathSub : public QGraphicsObject
{
    Q_OBJECT
public:
    explicit BlendPathSub(int width, int height, Blender * blender);

    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	PropertyMap property;

protected:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent * event);

private:
    Blender * blender;
    QRectF m_geometry;

signals:
    
public slots:
    
};
