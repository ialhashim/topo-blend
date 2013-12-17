#pragma once
#include "transform3d.h"
#include "Geometry.h"
#include "StructureGraph.h"
#include "GraphCorresponder.h"

static QString TRANS = "TRANS";
static QString REF = "REF";
static QString PARALLEL = "PARALLEL";
static QString ORTHOGONAL = "ORTHOGONAL";
static QString COPLANAR = "COPLANAR";
static QString CONNECTED = "CONNECTED"; // for all connected pairs
static QString NONE = "NONE";

class PairRelation
{
public:
	PairRelation():n1(0),n2(0),tag(true),type(NONE){}
	PairRelation(Structure::Node* node1, Structure::Node* node2):n1(node1),n2(node2){}
	bool isDegenerated(){return n1==0 || n2==0;}
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

static QString TRANS_SYMMETRY = "TRANS_SYMMETRY"; // build from Trans pair, except AXIS_SYMMETRY
static QString AXIS_SYMMETRY = "AXIS_SYMMETRY";
static QString REF_SYMMETRY = "REF_SYMMETRY";

struct GroupRelation
{
	GroupRelation():deviation(0.0){}

    QVector<QString> ids; //QSet<QString> ids;
    QString type;
	QString note;// from trans or ref pair
    double diameter; // group diameter
    double deviation; // absolute mean deviation
    bool tag;

    Point_3 center; // for AXIS_SYMMETRY
    Vector_3 direction; // normalized. line direction for AXIS_SYMMETRY
    
    Eigen::Vector3d normal;// normalized. for coplanar & REF_SYMMETRY group
    Eigen::Vector3d point;// for coplanar group & REF_SYMMETRY group

    bool equal(GroupRelation& gr);
    friend QTextStream& operator<<(QTextStream& os, const GroupRelation& gr);
};

bool isTaged(const PairRelation &pr);
bool isTagedGr(const GroupRelation &gr);
void saveToFile(QString filename, QVector<PairRelation>& prs);
void saveToFile(QString filename, QVector<GroupRelation>& grs);
template<class T>
int countByType(QVector<T>& prs, QString& type)
{
    int i(0);
    foreach(T pr, prs)
    {
        if ( 0 == type.compare(pr.type))
            ++i;
    }
    return i;
}






class RelationDetector
{
public:
	RelationDetector(Structure::Graph* g, const QString& logprefix, int ith, double normalizeCoef, int pointLevel=1, int logLevel=0);
	~RelationDetector(){logFile_.close();}
	
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
	// relative to the diameter of the graph, i.e. model
    double computeGroupDiameter(GroupRelation& gr);
	void computeGroupCenter(GroupRelation& gr);
	void findRefPlane(Structure::Node* n1, Structure::Node* n2, Eigen::Vector3d& center,Eigen::Vector3d& normal);
	double fixDeviationByPartName(const QString& s1, const QString& s2, double deviation, double times=0.5);

    // we do not order part in a group by ref plane
    // return mean deviation of the group
	// precondition: none
    double computeRefSymmetryGroupDeviation(GroupRelation& gr, int pointLevel);
    // order part in a group by angle around the axis
    // return mean deviation of the group
	// precondition: center & direction of gr is known.
    double computeAxisSymmetryGroupDeviation(GroupRelation& gr, int pointLevel);
    // compute the center and direction of the group
    void computeTransGroupInfo(GroupRelation &gr, QSet<QString>& ids);

	static Structure::Link* findLink(Structure::Node *n1, Structure::Node *n2, Structure::Graph * graph);
	static double computeDeviationByLink(Structure::Link* link);
protected:
	Eigen::Vector3d computeCptsCenter(Structure::Node* nn);
    double errorOfRefSymmGroup(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& center, Eigen::Vector3d& normal, int pointLevel);
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
	double normalizeCoef_;
	int pointLevel_;// 0 for main control points, 1 for all control points, 2 for all points.
protected:
	QFile logFile_;
	QTextStream logStream_;
	int logLevel_;	// 0 for no log; 1 for coarse log; 2 for detailed log
};
