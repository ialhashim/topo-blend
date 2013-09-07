#pragma once

#include "DemoPage.h"
#include "GraphCorresponder.h"

// Forward Declare
class TopoBlender; class Scheduler;

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
    
	int numSuggestions;

signals:
    void blendPathsReady();

public slots:
    void show();
    void hide();

	void setGraphCorresponder(GraphCorresponder *);
	void preparePaths();
	void computeBlendPaths();

private:
	QVector<BlendPath> blendPaths;
	void computePath(int index);
	GraphCorresponder * m_gcorr;
};
