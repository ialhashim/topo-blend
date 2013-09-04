#pragma once
#include <qglviewer/camera.h>
#include "DemoGlobal.h"

class Scene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit Scene(QObject *parent = 0);

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
    qglviewer::Camera * camera;
    void setupCamera();

signals:
    void wheelEvents(QGraphicsSceneWheelEvent*);

public slots:
    
};
