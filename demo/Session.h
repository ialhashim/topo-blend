#pragma once

#include "DemoGlobal.h"
#include "Scene.h"
#include "ShapesGallery.h"
#include "Controls.h"
#include "Matcher.h"
#include "Blender.h"

class Session : public QObject
{
    Q_OBJECT
public:
    explicit Session(Scene * scene, ShapesGallery * gallery, Controls * control,
                     Matcher * matcher, Blender * blender, QObject *parent = 0);

    Scene * s;
    ShapesGallery * g;
    Controls * c;
    Matcher * m;
    Blender * b;

signals:
    void update();

public slots:
    void shapeChanged(int,QGraphicsItem*);

    void hideAll();
    void showSelect();
    void showMatch();
    void showCreate();
};
