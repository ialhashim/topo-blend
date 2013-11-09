#pragma once

#include <vector>
#include <algorithm>
#include <sstream>
#include <numeric>


#include "GraphCorresponder.h"
#include "transform3d.h"

class Scorer
{
public:
	Scorer(Structure::Graph* g, const QString& logprefix, int ith, int logLevel=0):graph_(g),logLevel_(logLevel)
	{
		if ( logLevel_ > 0)
		{			
			logFile_.setFileName(logprefix + QString::number(ith) + ".log");
			if (!logFile_.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
			logStream_.setDevice(&logFile_);
		}
	}
	~Scorer()
	{
		logFile_.close();
	}

	Structure::Graph* graph_;

protected:
	QFile logFile_;
	QTextStream logStream_;
	int logLevel_;	// 0 for no log; 1 for coarse log; 2 for detailed log
};

