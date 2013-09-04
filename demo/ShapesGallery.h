#pragma once

#include "scene.h"
#include "ShapeItem.h"

class ShapesGallery : public QObject
{
    Q_OBJECT
public:
    explicit ShapesGallery(Scene * scene);

    bool isVisible();

private:
    Scene * s;
    QVector<QGraphicsItem*> items;
    bool visible;

    QVector<QGraphicsItem*> listA, listB;
    int indexA, indexB;
    ShapeItem *makeShapeItem(QString name, PropertyMap info);

    void arrangeList(QVector<QGraphicsItem *> &list, int x);
    void scrollTo(QVector<QGraphicsItem *> &list, int & index);

signals:
    void shapeChanged(int,QGraphicsItem*);

public slots:
    void loadDataset(DatasetMap dataset);
    void layout();

    void hide();
    void show();

    void wheelEvent(QGraphicsSceneWheelEvent*);
};
