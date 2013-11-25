#pragma once
#include "transform3d.h"
#include "Geometry.h"
#include "StructureGraph.h"
#include "GraphCorresponder.h"

static QString TRANS = "TRANS";
static QString PARALLEL = "PARALLEL";
static QString REF = "REF";
static QString ORTHOGONAL = "ORTHOGONAL";
static QString COPLANAR = "COPLANAR";
static QString CONNECTED = "CONNECTED"; // for all connected pairs
static QString NONE = "NONE";

static QString TRANS_SYMMETRY = "TRANS_SYMMETRY"; // build from Trans pair, except AXIS_SYMMETRY
static QString AXIS_SYMMETRY = "AXIS_SYMMETRY";
static QString REF_SYMMETRY = "REF_SYMMETRY";

class PairRelation
{
public:
	PairRelation():n1(0),n2(0),tag(false),type(NONE){}
	PairRelation(Structure::Node* node1, Structure::Node* node2):n1(node1),n2(node2){}
	bool isDegenerated()
	{
		return n1==0 || n2==0;
	}
	Structure::Node* n1;
	Structure::Node* n2;
	QString type;

	//for connected pair, deviation is: mini distance between the skeleton' s control points of n1 & n2. has been normalized by graph->bbox().diagonal().norm()
	//for translation pair, deviation is: mean distance between the skeleton' s control points of n1 & that of n2 after translation. has been normalized by n1 & n2's diameters
	double deviation;

	bool tag;
    double diameter; // diameter of the pair

    Eigen::Vector3d trans_vec;// for TRANS
	Eigen::Vector3d normal; // for coplanar
    Eigen::Vector3d point; // for coplanar

    friend QTextStream& operator<<(QTextStream& os, const PairRelation& pr);
};


class RelationDetector
{
public:
	RelationDetector(Structure::Graph* g, const QString& logprefix, int ith, int logLevel=0):graph_(g),logLevel_(logLevel)
	{
		thRadiusRadio_ = 1.2;

        thTransRadio_ = 0.03; //1.3
        thRefRadio_ = 0.03;
        thAxisDeviationRadio_ = 0.9;
        thCopla_ = 0.002;//0.1
        thParal_ = 0.001;
        thOthog_ = 0.001;//0.1

        thAxisGroup_ = 0.01;
        thRefGroup_ = 0.01;
        thCoplaGroup_ = 0.003;


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
	
	//////////////////////////////////////////////////
	// find nodes in the source or target shape
	// bSource == true means that the id is from target shape, & we want to find its corresponded node in source shape, then graph should be source
	std::vector<Structure::Node*> findNodesInST(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource);
	// find nodes in a blended shape, all returned nodes are not degenerated.
	// bSource == true means that the id is from source shape
	std::vector<Structure::Node*> findNodesInB(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource);

	
	/////////////////////////////////////////////////
	Eigen::MatrixXd node2matrix(Structure::Node* node, int pointLevel);
	int extractCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, int pointsLevel);
	void vectorPts2MatrixPts(const std::vector<Eigen::Vector3d>& ptsin, Eigen::MatrixXd& ptsout);
	std::vector<Eigen::Vector3d> matrixPts2VectorPts(Eigen::MatrixXd& ptsin);


	////////////////////////////////////////////////
	static Segment_3 curve2segment(Structure::Node * n);
	static Line_3 curve2line(Structure::Node * n);
	static Eigen::Vector3d curve2vectorNormalized(Structure::Node * n);

	static std::vector<Point_3> sheet2rect(Structure::Node * n);
	static void sheet2plane(Structure::Sheet * s, Vector3& point, Vector3& normal);
	static bool node2direction(Structure::Node * n, Vector_3& result);

	// output:
    //        point
    //        normal
    // only work for two curve node
    void createPlane(Structure::Node *n1,Structure::Node *n2, Eigen::Vector3d& point, Eigen::Vector3d& normal);
	//////////////////////////////////////////////////
	double computePairDiameter(PairRelation& pr);
	double fixDeviationByPartName(QString& s1, QString& s2, double deviation);
public:
	double thRadiusRadio_;
    double thTransRadio_;
    double thRefRadio_;
    double thParal_;
    double thOthog_;
    double thCopla_;

    double thRefGroup_;
    double thAxisGroup_;
    double thAxisDeviationRadio_;
    double thCoplaGroup_;

	Structure::Graph* graph_;
protected:
	QFile logFile_;
	QTextStream logStream_;
	int logLevel_;	// 0 for no log; 1 for coarse log; 2 for detailed log
};
