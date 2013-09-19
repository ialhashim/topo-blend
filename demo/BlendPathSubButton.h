#pragma once

#include <QGraphicsObject>
#include "Blender.h"

class BlendPathSubButton : public QGraphicsObject
{
    Q_OBJECT
public:
    explicit BlendPathSubButton(int width, int height, Blender * blender, int z_order = 0);

	bool isOnTop();
    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	PropertyMap property;
	Blender * blender;

protected:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent * event);

private:
    QRectF m_geometry;

signals:
    
public slots:
    
};
