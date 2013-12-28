#pragma once

#include <QObject>
#include <QMap>
#include <QStack>
#include <vector>

#include "StructureGraph.h"

#include "GLVertex.h"

// Forward declare
class GraphCorresponder;
class Scheduler;
class TopoBlender;
typedef QMap<QString, QMap<QString, QVariant> > SynthData;

extern QStack<double> nurbsQuality;

static void inline beginFastNURBS(){
	nurbsQuality.clear();
	nurbsQuality.push(TIME_ITERATIONS);
	nurbsQuality.push(CURVE_TOLERANCE);
	nurbsQuality.push(RombergIntegralOrder);

	TIME_ITERATIONS			= 6;
	CURVE_TOLERANCE			= 1e-05;
	RombergIntegralOrder	= 5;
}

static void inline endFastNURBS(){
	if(nurbsQuality.size() < 3) return;
	RombergIntegralOrder = nurbsQuality.pop();
	CURVE_TOLERANCE = nurbsQuality.pop();
	TIME_ITERATIONS = nurbsQuality.pop();
}

class SynthesisManager : public QObject
{
	Q_OBJECT
public:
    SynthesisManager(GraphCorresponder * gcorr, Scheduler * scheduler, TopoBlender * blender, int samplesCount = 20000);
    
	GraphCorresponder * gcorr;
	Scheduler * scheduler;
	TopoBlender * blender;

	PropertyMap property;

	QVector<Structure::Graph*> graphs();
	Structure::Graph * graphNamed(QString graphName);

	// Synthesis data [graph][node][data]
	QMap<QString, QMap<QString, QMap<QString,QVariant> > > synthData;
	SynthData renderData;
	int samplesCount;

	std::vector<GLVertex> vertices;
	SynthData currentData;
	QMap<QString, QVariant> currentGraph;

	// Visualization
	QMap<QString, QMap<QString,QVariant> > sampled;

	// Options
	bool isSplatRenderer;
	double splatSize;
	float pointSize;
	QColor color;

	bool samplesAvailable(QString graph, QString nodeID);

public slots:
    void generateSynthesisData();
	void setSampleCount(int numSamples);
    void saveSynthesisData(QString parentFolder = "");
    void loadSynthesisData(QString parentFolder = "");
    void genSynData();
    void reconstructXYZ();
	void outputXYZ();
	void clear();

    void doRenderAll();
    void renderAll();
	void renderCurrent();
	void renderCurrent( Structure::Graph * currentGraph, QString path = "" );
	void renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false, bool isOutParts = true );

	void drawSampled();
	void geometryMorph( SynthData & data, Structure::Graph * graph, bool isApprox, int limit = -1 );
	void drawSynthesis( Structure::Graph * activeGraph );

	void bufferCleanup();

	void emitSynthDataReady();

signals:
	void setMessage(QString);
	void progressChanged(double);
	void synthDataReady();
	void updateViewer();
};
