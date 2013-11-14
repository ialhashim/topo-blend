#pragma once
#include "transform3d.h"

Eigen::MatrixXd node2matrix(Structure::Node* node, int pointLevel);

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
	
	// find nodes in the source or target shape
	// bSource == true means that the id is from target shape, & we want to find its corresponded node in source shape, then graph should be source
	std::vector<Structure::Node*> findNodesInST(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource);


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
		bSource_ = ith == 0;
		pointsLevel_ = 1;
	}
	// if bSource_, g is the target shape (graph_ is source shape). else g is the source shape
	void detect(Structure::Graph* g, QVector<PART_LANDMARK> &corres);
	
	QVector<PairRelationBasic> pairs_;
	bool bSource_;
	int pointsLevel_; // 0 for main control points, 1 for all control points, 2 for all points.
};
