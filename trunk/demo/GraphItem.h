#pragma once
#include <QStack>
#include <qglviewer/camera.h>
#include <QGraphicsScene>
#include <QGraphicsObject>
#include <QPropertyAnimation>

#include "StructureGraph.h"

class GraphItem : public QGraphicsObject{
    Q_OBJECT
    Q_PROPERTY(QRectF geometry READ boundingRect WRITE setGeometry NOTIFY geometryChanged)
public:
    GraphItem(Structure::Graph* graph, QRectF region);

    Structure::Graph* g;

    void setGeometry(QRectF newGeometry);
    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void draw3D(qglviewer::Camera * camera);

    QRectF popState();
    void pushState();
    QPropertyAnimation * animateTo(QRectF newRect, int duration = 200);

private:
    QRectF m_geometry;
    QStack<QRectF> states;

public slots:
    void geometryHasChanged();

signals:
    void geometryChanged();
};
