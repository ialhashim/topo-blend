#pragma once

#include "DemoGlobal.h"
#include "scene.h"
#include "ShapesGallery.h"

class Session : public QObject
{
    Q_OBJECT
public:
    explicit Session(Scene * scene, ShapesGallery * gallery, QObject *parent = 0);
    
    Scene * s;
    ShapesGallery * g;

signals:
    
public slots:
    void shapeChanged(int,QGraphicsItem*);
};
