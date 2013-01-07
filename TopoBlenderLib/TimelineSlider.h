#pragma once

#include <QGraphicsObject>

class TimelineSlider : public QGraphicsObject
{
	Q_OBJECT
public:
    TimelineSlider();

    QImage icon;

    // required QGraphicsItem members:
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	int myY;
	void forceY(int newY);

	// Time operations
	void reset();
	int currentTime();

	bool isEnabled;
	void enable();

protected:
    virtual QVariant itemChange ( GraphicsItemChange change, const QVariant & value );

public slots:

signals:
	void timeChanged(int);
};

