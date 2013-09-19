#pragma once
#include <QElapsedTimer>
#include "DemoPage.h"
#include "GraphCorresponder.h"
#include "TopoBlender.h"

// Forward Declare
class Scheduler; class ProgressItem; class SynthesisManager;
class BlendPathRenderer; class BlendRenderItem; class BlendPathSubButton;

// Blend path container
struct BlendPath{
	Structure::Graph * source;
	Structure::Graph * target;
	GraphCorresponder * gcorr;
	QSharedPointer<Scheduler> scheduler;
	QSharedPointer<TopoBlender> blender;
};

extern QVector< BlendPath > blendPaths;

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
	friend class BlendPathSubButton;
	friend class BlendPathSub;

signals:
    void blendPathsReady();
    void blendPathsDone();
	void blendStarted();
	void blendDone();

public slots:
    void show();
    void hide();
	void cleanUp();

	void setGraphCorresponder(GraphCorresponder *);
	void preparePaths();
	void synthDataReady();

    void runComputeBlendPaths();
	void computeBlendPaths();

    void progressChanged();
	void blenderDone();
	void blendResultDone(QGraphicsItem* item);
	void blenderAllResultsDone();

	void keyReleased(QKeyEvent* keyEvent);

private:
	QVector< QGraphicsItemGroup* > blendPathsItems;
	QVector< QVector< QSharedPointer<BlendRenderItem> > > resultItems;
	QVector< QVector< QSharedPointer<BlendPathSubButton> > > blendSubItems;
	QVector< QGraphicsItem* > auxItems;
	
	QSharedPointer<SynthesisManager> s_manager;

	bool isSample;
	bool isFinished;

    void computePath(const int &index);

	GraphCorresponder * m_gcorr;
	BlendPathRenderer * renderer;
	ProgressItem * progress;

	QElapsedTimer pathsTimer;
	QElapsedTimer synthTimer;
	QElapsedTimer blendTimer;
	QElapsedTimer renderTimer;
};