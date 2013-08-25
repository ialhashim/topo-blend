#pragma once

#include "topo-blend.h"
#include "Synthesizer.h"
#include "GLVertex.h"

class SynthesisManager : public QObject
{
	Q_OBJECT
public:
    SynthesisManager(topoblend * topo_blender) : tb(topo_blender) {}
    topoblend * tb;

	QVector<Structure::Graph*> graphs();
	Structure::Graph * graphNamed(QString graphName);

	void clear();

	// Synthesis data [graph][node][data]
	QMap<QString, QMap<QString, QMap<QString,QVariant> > > synthData;
	SynthData renderData;

	// Visualization
	QMap<QString, QMap<QString,QVariant> > sampled;
	std::vector<GLVertex> vertices;
	SynthData currentData;
	QMap<QString, QVariant> currentGraph;

	bool samplesAvailable(QString graph, QString nodeID);

public slots:
    void generateSynthesisData();
    void saveSynthesisData(QString parentFolder = "");
    void loadSynthesisData(QString parentFolder = "");
    void genSynData();
    void reconstructXYZ();
	void outputXYZ();

    void doRenderAll();
    void renderAll();
    void renderCurrent();
    void renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false );
    void draftRender();

	void drawSampled();
	void geometryMorph( SynthData & data, Structure::Graph * graph, bool isApprox, int limit = -1 );
	void drawSynthesis();
};
