#pragma once

#include <QGraphicsItem>
#include "TopoBlender.h"

// red, orange, yellow, green, blue
// pink, purpule, portage, lacoste, corn
static QColor TaskColors[] = { QColor(255,97,121),  QColor(107,255,135), 
	QColor(255,219,88), QColor(255,165,107) , QColor(104,126,255),
	QColor(242,5,135), QColor(113,53,242), QColor(138,109,242), QColor(3,166,60), QColor(242,203,5)};

class Task : public QGraphicsItem
{
public:
	enum TaskType{ SHRINK, MORPH, MERGE, SPLIT, GROW };

	Task( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID );
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent * event);

	void execute();

	void prepareGrowShrink();
	void executeGrowShrink();

	void prepareMorph();
	void executeMorph();

	void prepareSplitMerge();
	void executeSplitMerege();

	Structure::Node * node();

	// Task properties
	TaskType type;

	Structure::Graph *active, *target;
	QMap<QString, QVariant> property;

	// Time related
	double start;
	double length;
	double current;
	bool isReady;
	int taskID;

	void setLength(int newLength);
	void reset();
	bool stillWorking();

	// Visual properties
	int width;
	int height;
	QColor mycolor;

	// Resizing variables
	bool isResizing;
	int resizeDir;
	QPointF clickPos, myOldPos;
	int myOldWidth;

	// DEBUG:
	std::vector<Vec3d> debugPoints, debugPoints2;
	void drawDebug();

protected:
	virtual QVariant itemChange ( GraphicsItemChange change, const QVariant & value );

};
