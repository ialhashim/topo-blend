#pragma once
#include <QElapsedTimer>
#include "DemoPage.h"
#include "GraphCorresponder.h"

// Forward Declare
class TopoBlender; class Scheduler; class ProgressItem; class SynthesisManager;
class BlendPathRenderer;

// Blend path container
struct BlendPath{
	Structure::Graph * source;
	Structure::Graph * target;
	GraphCorresponder * gcorr;
	QSharedPointer<Scheduler> scheduler;
	QSharedPointer<TopoBlender> blender;
	bool isReady;
};

class Blender : public DemoPage
{
    Q_OBJECT
public:
    explicit Blender(Scene * scene, QString title);
    
	int numSuggestions, numInBetweens;
	int itemHeight;
	int graphItemWidth;

	friend class BlendPathRenderer;

signals:
    void blendPathsReady();
	void allPathsDone();

public slots:
    void show();
    void hide();
	void cleanUp();

	void setGraphCorresponder(GraphCorresponder *);
	void preparePaths();
	void synthDataReady();
	void computeBlendPaths();

	void progressChanged();
	void singlePathDone();
	void blenderDone();
	void blendResultDone(QGraphicsItem* item);

	void keyReleased(QKeyEvent* keyEvent);

private:
	QVector<BlendPath> blendPaths;
	QVector<QGraphicsItemGroup*> blendPathsItems;
	QVector<QGraphicsItem*> resultItems;
	QVector<QGraphicsItem*> auxItems;
	SynthesisManager * s_manager;

	bool isSample;
	bool isFinished;

	void computePath(int index);

	GraphCorresponder * m_gcorr;
	BlendPathRenderer * renderer;
	ProgressItem * progress;

	QElapsedTimer pathsTimer;
	QElapsedTimer synthTimer;
	QElapsedTimer blendTimer;
	QElapsedTimer renderTimer;
};
