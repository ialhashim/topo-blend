#pragma once

#include <Eigen/Core>

#include "StructureCurve.h"
#include "StructureSheet.h"

#include "DynamicVoxel.h"
using namespace DynamicVoxelLib;

// DEBUG:
#include "../CustomDrawObjects.h"

#include "qglviewer/camera.h"

// Relink
typedef QMap<Structure::Link*,Vector3> LinksDelta;
Q_DECLARE_METATYPE( LinksDelta );

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
	void removeEdge( QString n1_id, QString n2_id );

	QString linkName( QString n1_id, QString n2_id );
    QString linkName( Node * n1, Node * n2 );

    // Accessors
    Node* getNode(QString nodeID);
	Link* getEdge(QString linkID);
	Link* getEdge(QString id1, QString id2);
	Vector3 nodeIntersection( Node * n1, Node * n2 );
	Curve* getCurve(Link * l);
	QVector<Link*> getEdges( QString nodeID );
	QMap< Link*, std::vector<Vec4d> > linksCoords( QString nodeID );
	QVector<Link*> nodeEdges( QString nodeID );
	QList<Link*> furthermostEdges( QString nodeID );
	Vector3 position( QString nodeID, Vec4d coord );
	void replaceCoords( QString nodeA, QString nodeB, std::vector<Vec4d> coordA, std::vector<Vec4d> coordB );
	int indexOfNode( Node * node );

	// Input / Output
	void saveToFile(QString fileName) const;
	void loadFromFile(QString fileName);

	// Visualization
    void draw();
	void drawAABB();
    void draw2D(int width, int height);
	QImage fontImage;

	// Synthesis
	void materialize( SurfaceMesh::Model * m, Scalar voxel_scaling = 1.0 );
	DynamicVoxel::QuadMesh cached_mesh;

    // Analysis
    Node * rootBySize();
    Node * rootByValence();
	int numCanVisit( Structure::Node * node );
	bool isConnected();
	bool isCutNode(QString nodeID);
	bool isBridgeEdge( Structure::Link * link );

	// Modifier
	void moveBottomCenterToOrigin();
	void normalize();
	void rotate(double angle, Vector3 axis);
	void scale(double scaleFactor);

	// Point Landmarks
	QVector<POINT_ID> selectedControlPointsByColor(QColor color);
	void clearSelections();

	// DEBUG:
	std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
	void printAdjacency();
	VectorSoup vs;
	PolygonSoup ps;
	void printLinksInfo();
};

}

Q_DECLARE_METATYPE(SurfaceMesh::Model *)
Q_DECLARE_METATYPE(Structure::Graph *)
