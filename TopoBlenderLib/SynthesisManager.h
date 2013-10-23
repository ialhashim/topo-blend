#pragma once

#include <vector>
#include <QObject>
#include <QMap>

#include "GLVertex.h"

#include "StructureGraph.h"

// Forward declare
class GraphCorresponder;
class Scheduler;
class TopoBlender;
typedef QMap<QString, QMap<QString, QVariant> > SynthData;

static inline void beginFastNURBS();
static inline void endFastNURBS();

class SynthesisManager : public QObject
{
	Q_OBJECT
public:
    SynthesisManager(GraphCorresponder * gcorr, Scheduler * scheduler, TopoBlender * blender, int samplesCount = 20000);
    
	GraphCorresponder * gcorr;
	Scheduler * scheduler;
	TopoBlender * blender;

	QVector<Structure::Graph*> graphs();
	Structure::Graph * graphNamed(QString graphName);

	// Synthesis data [graph][node][data]
	QMap<QString, QMap<QString, QMap<QString,QVariant> > > synthData;
	SynthData renderData;
	int samplesCount;

	// Visualization
	QMap<QString, QMap<QString,QVariant> > sampled;
	std::vector<GLVertex> vertices;
	SynthData currentData;
	QMap<QString, QVariant> currentGraph;

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
	void renderCurrent( Structure::Graph * currentGraph, QString path = "" );
	void renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false );

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
