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

public slots:
    void show();
    void hide();

    void visualize();

signals:
	void corresponderCreated(GraphCorresponder *);
	void correspondenceFromFile();
};
