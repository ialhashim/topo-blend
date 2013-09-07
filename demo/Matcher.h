#pragma once

#include "DemoPage.h"
#include "GraphCorresponder.h"

class Matcher : public DemoPage
{
    Q_OBJECT
public:
    explicit Matcher(Scene * scene, QString title);
    
    GraphCorresponder *gcorr;

	QVector<QColor> coldColors, warmColors;

signals:
    
public slots:
    void show();
    void hide();

    void visualize();
};
