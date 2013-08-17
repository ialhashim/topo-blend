#pragma once
#include "topo-blend.h"

class SynthesisManager
{
public:
    SynthesisManager(topoblend * topo_blender) : tb(topo_blender) {}
    topoblend * tb;

public slots:
    void generateSynthesisData();
    void saveSynthesisData(QString parentFolder = "");
    void loadSynthesisData(QString parentFolder = "");
    void outputPointCloud();
    void genSynData();
    void reconstructXYZ();
    void combineMeshesToOne();

    void doRenderAll();
    void renderAll();
    void renderCurrent();
    void renderGraph( Structure::Graph graph, QString filename, bool isOutPointCloud, int reconLevel, bool isOutGraph = false );
    void draftRender();
};
