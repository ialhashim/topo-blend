#pragma once

#include "DemoPage.h"
#include "ShapeItem.h"

class ShapesGallery : public DemoPage
{
    Q_OBJECT
public:
    explicit ShapesGallery(Scene * scene, QString title);

private:
    QVector<QGraphicsItem*> listA, listB;
    int indexA, indexB;
	ShapeItem *makeShapeItem( QString name, PropertyMap info, int idx, bool isRight );
    void arrangeList(QVector<QGraphicsItem *> &list, int x);
    void scrollTo(QVector<QGraphicsItem *> &list, int & index);

signals:
    void shapeChanged(int,QGraphicsItem*);

public slots:
    void show();
    void hide();

    void loadDataset(DatasetMap dataset);
    void layout();

    void wheelEvent(QGraphicsSceneWheelEvent*);
	void scrollToItem(ShapeItem* item);
};
