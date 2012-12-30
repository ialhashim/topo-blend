#pragma once

#include <QGraphicsItem>
#include "TopoBlender.h"

static QColor TaskColors[] = { Qt::red, Qt::yellow, Qt::green, Qt::cyan, Qt::blue };

class Task : public QGraphicsItem
{
public:
	enum TaskType{ SHRINK, SPLIT, MERGE, GROW, MORPH };

	Task( Structure::Graph * graph, TaskType taskType );
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);

	// Task properties
	Structure::Graph * active;
	TaskType type;
	int start;
	int length;
	QMap<QString, QVariant> property;

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
