#pragma once

#include "DemoPage.h"
#include "GraphCorresponder.h"

// Forward Declare
class TopoBlender; class Scheduler; class ProgressItem; class SynthesisManager;

// Blend path container
struct BlendPath{
	Structure::Graph * source;
	Structure::Graph * target;

	GraphCorresponder * gcorr;
	TopoBlender * blender;
	Scheduler * scheduler;
};

class Blender : public DemoPage
{
    Q_OBJECT
public:
    explicit Blender(Scene * scene, QString title);
    
	int numSuggestions, numInBetweens;

signals:
    void blendPathsReady();
	void allPathsDone();

public slots:
    void show();
    void hide();

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
	SynthesisManager * s_manager;

	void computePath(int index);

	GraphCorresponder * m_gcorr;
	ProgressItem * progress;
};
