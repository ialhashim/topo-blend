#pragma once

#include <QGraphicsObject>
#include "SpinnerItem.h"

class ProgressItem : public QGraphicsObject
{
    Q_OBJECT
public:
    explicit ProgressItem(QString message, bool isLoading, QGraphicsScene * scene);
    ~ProgressItem();

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
    QGraphicsScene * scene;
    QGraphicsItemGroup * items;
    SpinnerItem * spinner;
    QString message, oldMessage;
    QRectF messageRect;
    bool isLoading;
    double progress;
    bool isSmoothAnimation;

signals:
    
public slots:
    void startProgress();
    void setProgress(double);
    void stopProgress();

    void applyProgress();

    void show();
    void close();
    void visiblityChanged();
};
