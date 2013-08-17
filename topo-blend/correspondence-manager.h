#pragma once
#include "topo-blend.h"

class CorrespondenceManager
{
public:
    CorrespondenceManager(topoblend * topo_blender) : tb(topo_blender) {}
    topoblend * tb;
};
