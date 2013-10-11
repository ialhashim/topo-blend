#pragma once

#include <QGraphicsObject>
#include <QMap>

#include "StructureGraph.h"

class BlendRenderItem : public QGraphicsObject 
{
	Q_OBJECT
	Q_PROPERTY(QPointF pos READ pos WRITE setPos)

public:
    BlendRenderItem(QPixmap pixmap);

	QPixmap pixmap;
	QMap<QString, QVariant> property;

    QRectF boundingRect() const { return pixmap.rect(); }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
		
	Structure::Graph * graph();

protected:
	virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event);
	bool isUnderMouse();
signals:
	void doubleClicked(BlendRenderItem*);
};
