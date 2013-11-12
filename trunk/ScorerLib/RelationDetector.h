#pragma once
#include "transform3d.h"

struct PairRelationBasic
{
	PairRelationBasic():n1(0),n2(0)
	{
	}
	bool isDegenerated()
	{
		return n1==0 || n2==0;
	}
	Structure::Node* n1;
	Structure::Node* n2;
	double miniDist;//mini distance between the skeleton' s control points of n1 & n2. has been normalized by graph->bbox().diagonal().norm()

    friend QTextStream& operator<<(QTextStream& os, const PairRelationBasic& pr);
};


class RelationDetector
{
public:
	RelationDetector(Structure::Graph* g, const QString& logprefix, int ith, int logLevel=0):graph_(g),logLevel_(logLevel)
	{
		thIntersectDist_ = 0.1;

		if ( logLevel_ > 0)
		{			
			logFile_.setFileName(logprefix + QString::number(ith) + ".log");
			if (!logFile_.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
			logStream_.setDevice(&logFile_);
		}
	}
	~RelationDetector()
	{
		logFile_.close();
	}

	Structure::Graph* graph_;

	double thIntersectDist_;

protected:
	QFile logFile_;
	QTextStream logStream_;
	int logLevel_;	// 0 for no log; 1 for coarse log; 2 for detailed log
};

class ConnectedPairDetector : public RelationDetector
{
public:
	ConnectedPairDetector(Structure::Graph* g, int ith, int logLevel=0):RelationDetector(g, "ConnectedPairDetector-", ith, logLevel)
	{
	}
	void detect();
	
	QVector<PairRelationBasic> pairs_;
};
