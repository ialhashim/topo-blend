#pragma once
#include <QStack>
#include <qglviewer/camera.h>
#include <QGraphicsScene>
#include <QGraphicsObject>
#include <QPropertyAnimation>

#include "StructureGraph.h"
#include "DemoGlobal.h"

class GraphItem : public QGraphicsObject{
    Q_OBJECT
    Q_PROPERTY(QRectF geometry READ boundingRect WRITE setGeometry NOTIFY geometryChanged)
public:
	GraphItem(Structure::Graph *graph, QRectF region, qglviewer::Camera * camera);
    Structure::Graph* g;
	QString name;

    void setGeometry(QRectF newGeometry);
    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	QRectF setCamera();

    void draw3D();
	void pick(int x, int y);

    QRectF popState();
    void pushState();
    QPropertyAnimation * animateTo(QRectF newRect, int duration = 200);

private:
    QRectF m_geometry;
    QStack<QRectF> states;
	qglviewer::Camera * camera;

	// DEBUG:
	PointSoup ps1, ps2;
	VectorSoup vs1, vs2;
	SphereSoup ss1, ss2;

public slots:
    void geometryHasChanged();

signals:
    void geometryChanged();
	void pickResult( PropertyMap );
	void hit(GraphItem *, QString, Vector3);
};
