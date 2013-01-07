#pragma once

#include <Eigen/Core>

#include "StructureCurve.h"
#include "StructureSheet.h"

#include "DynamicVoxel.h"
using namespace DynamicVoxelLib;

namespace Structure{

typedef QPair<Node*,Node*> QPairNodes;
typedef QPair<Link*,Link*> QPairLink;
typedef QPair<Scalar, QPairLink> ScalarLinksPair;

struct Graph
{
	// Properties
    QVector<Node*> nodes;
    QVector<Link*> edges;
    QBox3D bbox();
    QMap< QString, QVariant > property;
	Eigen::MatrixXd adjacency;
	QMap< QString, void* > misc;

	int valence(Node * n);
	
	// Constructors
	Graph();
	Graph(QString fileName);
	Graph(const Graph & other);
    ~Graph();
	
	// Modifiers
    Node * addNode( Node * n );
	Node * removeNode( QString nodeID );

    Link * addEdge( Node * n1, Node * n2 );
	Link * addEdge( Node *n1, Node *n2, std::vector<Vec4d> coord1, std::vector<Vec4d> coord2, QString linkName = "" );
	Link * addEdge( QString n1_id, QString n2_id );
	void removeEdge( Node * n1, Node * n2 );

	QString linkName( QString n1_id, QString n2_id );
    QString linkName( Node * n1, Node * n2 );

    // Accessors
    Node* getNode(QString nodeID);
	Link* getEdge(QString id1, QString id2);
	Vector3 nodeIntersection( Node * n1, Node * n2 );
	Curve* getCurve(Link * l);
	QVector<Link*> getEdges( QString nodeID );
	QMap< Link*, std::vector<Vec4d> > linksCoords( QString nodeID );
	QVector<Link*> nodeEdges( QString nodeID );
	QList<Link*> furthermostEdges( QString nodeID );
	Vector3 position( QString nodeID, Vec4d coord );
	void replaceCoords( QString nodeA, QString nodeB, std::vector<Vec4d> coordA, std::vector<Vec4d> coordB );
	
	// Input / Output
	void saveToFile(QString fileName) const;
	void loadFromFile(QString fileName);

	// Visualization
    void draw();
    void draw2D(int width, int height);
	QImage fontImage;

	// Synthesis
	void materialize( SurfaceMeshModel * m, Scalar voxel_scaling = 1.0 );
	DynamicVoxel::QuadMesh cached_mesh;

    // Analysis
    Node * rootBySize();
    Node * rootByValence();
	bool isCutNode(QString nodeID);

	// Modifier
	void moveBottomCenterToOrigin();
	void normalize();

	// DEBUG:
	std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
	void printAdjacency();
};

}

Q_DECLARE_METATYPE(Structure::Graph *)
