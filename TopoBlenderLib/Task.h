#pragma once

#include <QGraphicsItem>
#include "TopoBlender.h"

class Task : public QGraphicsItem
{
public:
    Task(Structure::Node * assignedNode);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);

    enum TaskType{ GROW, SHRINK, SPLIT, CLONE };

	// Task properties
	TaskType type;
	Structure::Node * node;
	int start;
	int length;

	void setLength(int newLength);

	// Visual properties
	int width;
	int height;
	QColor mycolor;

	// Resizing variables
	bool isResizing;
	int resizeDir;
	QPointF clickPos, myOldPos;
	int myOldWidth;

protected:
	virtual QVariant itemChange ( GraphicsItemChange change, const QVariant & value );

};
