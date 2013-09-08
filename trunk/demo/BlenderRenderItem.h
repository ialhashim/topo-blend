#pragma once

#include <QGraphicsPixmapItem>
#include <QMap>

class BlenderRenderItem : public QGraphicsPixmapItem {
public:
	BlenderRenderItem(QPixmap pixmap) : QGraphicsPixmapItem(pixmap) {  }
	int pathID, blendIDX;
	QMap<QString, QVariant> property;
};
