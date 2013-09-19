#pragma once

#include "BlendPathSubButton.h"

class BlendPathSub : public QGraphicsObject
{
    Q_OBJECT
public:
    BlendPathSub(int x, int y, int height, int count, BlendPathSubButton * origin);

    QRectF boundingRect() const { return m_geometry; }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	void setup();

	BlendPathSubButton * origin;

protected:

private:
    int height;
    int count;
    QRectF m_geometry;
	QTimer *timer;
	QWidget *viewer;

	QVector<QGraphicsObject*> items;

public slots:
	void hideWhenInactive();
};
