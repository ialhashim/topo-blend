#pragma once

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
};

class Blender : public DemoPage
{
    Q_OBJECT
public:
    explicit Blender(Scene * scene, QString title);
    
	int numSuggestions, numInBetweens;
	int itemHeight;

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
	void pathDone();
	void blenderDone();
	void blendResultDone(QGraphicsItem* item);

private:
	QVector<BlendPath> blendPaths;
	QVector<QGraphicsItemGroup*> blendPathsItems;
	QVector<QGraphicsItem*> resultItems;
	SynthesisManager * s_manager;

	void computePath(int index);

	GraphCorresponder * m_gcorr;
	BlendPathRenderer * renderer;
	ProgressItem * progress;
};
