#pragma once
#include "scorer.h"
class ConnectivityScorer :
	public Scorer
{
public:
	ConnectivityScorer(Structure::Graph* g, int ith, int logLevel=0):Scorer(g, "ConnectivityScorer-", ith, logLevel)
    {
    }
};

