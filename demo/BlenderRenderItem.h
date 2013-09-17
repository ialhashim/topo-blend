#pragma once

#include <QGraphicsObject>
#include <QMap>

class BlenderRenderItem : public QGraphicsObject {
	Q_OBJECT
public:
    BlenderRenderItem(QPixmap pixmap);

	int pathID, blendIDX;
	QPixmap pixmap;
	QMap<QString, QVariant> property;

    QRectF boundingRect() const { return pixmap.rect(); }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};
