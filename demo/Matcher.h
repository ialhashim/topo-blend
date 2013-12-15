#pragma once

#include "DemoPage.h"
#include "GraphCorresponder.h"
#include "GraphItem.h"

enum MatcherState{ CLEARED, SINGLE, BOTH };
enum GraphHitMode{ MARKING, STROKES };

struct GraphHit{ 
	QString nid; Eigen::Vector3d p; 
	GraphHit(QString nid = "", Eigen::Vector3d p = Eigen::Vector3d(0,0,0)) : nid(nid), p(p){}
	bool operator==(const GraphHit &other) { return nid == other.nid; }
	operator QString() const { return nid; }
};

class Matcher : public DemoPage
{
    Q_OBJECT
public:
    explicit Matcher(Scene * scene, QString title);
    
    GraphCorresponder *gcorr;

	QVector<QColor> coldColors, warmColors;
	int c_cold, c_warm;

	QVector<PART_LANDMARK> prevCorrAuto, prevCorrManual;

	QColor curStrokeColor;

private:
	MatcherState state;
	QVector<GraphHit> groupA, groupB;
	GraphItem * prevItem;
	
	bool isAuto;
	bool isGrouping;

public slots:
    void show();
    void hide();

    void visualize();
	void resetColors();

	void graphHit( GraphItem::HitResult );

	void autoMode();
	void manualMode();
	void groupingMode();

	void setMatch();
	void clearMatch();

	void mousePress( QGraphicsSceneMouseEvent* mouseEvent );
	void mouseRelease( QGraphicsSceneMouseEvent* mouseEvent );
	void keyReleased( QKeyEvent* keyEvent );

signals:
	void corresponderCreated(GraphCorresponder *);
	void correspondenceFromFile();

	void switchedToManual();
};
