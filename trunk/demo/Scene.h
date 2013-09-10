#pragma once
#include "DemoGlobal.h"
#include "GraphItem.h"

class Scene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit Scene(QObject *parent = 0);

	GraphItem * inputGraphs[2];
	qglviewer::Camera * camera;

    bool isInputReady() { return inputGraphs[0] && inputGraphs[1]; }

    QRect graphRect(bool isRight);

protected:
    void drawBackground ( QPainter * painter, const QRectF & rect );
    void drawForeground ( QPainter * painter, const QRectF & rect );
    void draw3D();

    void testScene(); // delete this later

protected:
    void mousePressEvent( QGraphicsSceneMouseEvent * event );
    void mouseMoveEvent( QGraphicsSceneMouseEvent * event );
    void wheelEvent(QGraphicsSceneWheelEvent *event);

private:
    void setupCamera();
    void setupLights();

signals:
    void wheelEvents(QGraphicsSceneWheelEvent*);

public slots:
    
};
