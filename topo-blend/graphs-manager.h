#pragma once
#include "topo-blend.h"

class GraphsManager
{
public:
    GraphsManager(topoblend * topo_blender) : tb(topo_blender) {}
    topoblend * tb;

public slots:
    void loadModel();
    void saveModel();
    void modifyModel();
    void quickAlign();
    void normalizeAllGraphs();
    void clearGraphs();

    QString loadJobFileName();
};
