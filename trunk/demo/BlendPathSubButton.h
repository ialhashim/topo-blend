#pragma once

#include <QGraphicsObject>
#include "Blender.h"

class BlendPathSubButton : public QGraphicsObject
{
    Q_OBJECT
	Q_PROPERTY(QPointF pos READ pos WRITE setPos)

public:
    explicit BlendPathSubButton(int width, int height, Blender * blender, int z_order = 0);

    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	PropertyMap property;
	Blender * blender;

protected:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent * event);
	bool isUnderMouse();

private:
    QRectF m_geometry;

signals:
    
public slots:
    
};
