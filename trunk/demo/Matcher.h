#pragma once

#include "DemoPage.h"
#include "GraphCorresponder.h"
#include "GraphItem.h"

enum MatcherState{ CLEARED, SINGLE, BOTH };

class Matcher : public DemoPage
{
    Q_OBJECT
public:
    explicit Matcher(Scene * scene, QString title);
    
    GraphCorresponder *gcorr;

	QVector<QColor> coldColors, warmColors;
	int c_cold, c_warm;

private:
	MatcherState state;
	QVector<QString> groupA, groupB;
	GraphItem * prevItem;

public slots:
    void show();
    void hide();

    void visualize();
	void resetColors();

	void graphHit( GraphItem::HitResult );

	void autoMode();
	void manualMode();

	void setMatch();
	void clearMatch();

signals:
	void corresponderCreated(GraphCorresponder *);
	void correspondenceFromFile();
};
