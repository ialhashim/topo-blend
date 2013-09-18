#pragma once
#include <QElapsedTimer>
#include "DemoPage.h"
#include "GraphCorresponder.h"

// Forward Declare
class TopoBlender; class Scheduler; class ProgressItem; class SynthesisManager;
class BlendPathRenderer; class BlenderRenderItem; class BlendPathSub;

// Blend path container
struct BlendPath{
	Structure::Graph * source;
	Structure::Graph * target;
	GraphCorresponder * gcorr;
	QSharedPointer<Scheduler> scheduler;
	QSharedPointer<TopoBlender> blender;
};

class Blender : public DemoPage
{
    Q_OBJECT
public:
    explicit Blender(Scene * scene, QString title);
    void setupBlendPathItems();

	int numSuggestions, numInBetweens;
	int itemHeight;
	int graphItemWidth;

	friend class BlendPathRenderer;
	friend class BlendPathSub;

signals:
    void blendPathsReady();
	void allPathsDone();
	void blendStarted();
	void blendDone();

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
	void blenderAllResultsDone();

	void keyReleased(QKeyEvent* keyEvent);

private:
	QVector< BlendPath > blendPaths;
	QVector< QGraphicsItemGroup* > blendPathsItems;
	QVector< QVector< QSharedPointer<BlenderRenderItem> > > resultItems;
	QVector< QVector< QSharedPointer<BlendPathSub> > > blendSubItems;
	QVector< QGraphicsItem* > auxItems;
	
	QSharedPointer<SynthesisManager> s_manager;

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
