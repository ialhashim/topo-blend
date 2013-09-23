#pragma once
#include <QObject>
#include <QMap>
#include <QVector>
#include <QStack>
#include <QString>
#include <QVariant>
#include <QDebug>
#include <QElapsedTimer>
#include <QtConcurrentRun>
#include <QDir>
#include <QFileInfo>
#include <QtOpenGL>
#include <QGLWidget>
#include <QPainter>

#include <QGraphicsScene>
#include <QGraphicsItem>

typedef QMap<QString, QVariant> PropertyMap;
typedef QMap<QString, PropertyMap> DatasetMap;

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

static inline QImage colorize(const QImage & source, QColor color, double extraLight = 1.0){
	int w = source.width();
	int h = source.height();
	QImage newImage(w, h, QImage::Format_ARGB32);

	for(int x = 0; x < w; x++){
		for(int y = 0; y < h; y++){
			QRgb c = source.pixel(x,y);
			QColor oldColor(c);
			oldColor.setAlpha(qAlpha(c));
			QColor newColor = QColor::fromHsv(color.hue(), color.saturation(), qMin(255, qMax(0, int(oldColor.value() * extraLight))), oldColor.alpha());
			newImage.setPixel( x, y, newColor.rgba() );
		}
	}

	return newImage;
}
