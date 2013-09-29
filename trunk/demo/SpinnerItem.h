#pragma once
#include <QGraphicsItem>
#include <QColor>
#include <QPainter>
#include <QTimer>

class SpinnerItem : public QGraphicsItem {
public:
    SpinnerItem(int radius, int lineWidth, QColor color, int spanAngle = 60) :
        width(radius), height(radius), lineWidth(lineWidth), color(color), spanAngle(spanAngle), timer(new QTimer){}

    int width, height, lineWidth, spanAngle;
    QColor color;
    QTimer * timer;

    QRectF boundingRect() const { return QRectF(0,0,width,height); }
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        Q_UNUSED(option)
        Q_UNUSED(widget)

		int radius = width * 0.5;

		painter->translate(1,1);
		painter->setPen(QPen(QColor(0,0,0,60), lineWidth));
		painter->drawEllipse(boundingRect().center(), radius, radius);
		painter->translate(-1,-1);

        QColor backColor = color;
        backColor.setAlphaF(0.15);

        painter->setPen(QPen(backColor, lineWidth));
        painter->drawEllipse(boundingRect().center(), radius, radius);

        int startAngle = 90 - (spanAngle * 0.5);

        painter->setPen(QPen(color, lineWidth));
        painter->drawArc(boundingRect(), startAngle * 16, spanAngle * 16);
    }
};
