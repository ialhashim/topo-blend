#pragma once

#include "DynamicVoxel.h"

#include "StructureCurve.h"
#include "StructureSheet.h"

#include <Eigen/Core>

namespace Structure{

struct Graph
{
	// Properties
    QVector<Node*> nodes;
    QVector<Link> edges;
    QBox3D bbox();
    QMap< QString, QVariant > property;
	Eigen::MatrixXd adjacency;

	int valence(Node * n);
	
	// Constructors
	Graph();
	Graph(QString fileName);
    ~Graph();
	
	// Modifiers
    Node * addNode( Node * n );
    Link addEdge( Node * n1, Node * n2 );
	Link addEdge( Node * n1, Node * n2, Vec2d coord1, Vec2d coord2, QString linkName);
	Link addEdge(QString n1_id, QString n2_id);
	void removeEdge( Node * n1, Node * n2 );

    QString linkName( Node * n1, Node * n2 );

    // Accessors
    Node* getNode(QString nodeID);
	Link* getEdge(QString id1, QString id2);
	Vector3 nodeIntersection( Node * n1, Node * n2 );
	Curve* getCurve(Link * l);
	QMap<Link*, Vec2d> linksCoords( QString nodeID );
	
	// Input / Output
	void saveToFile(QString fileName);
	void loadFromFile(QString fileName);

	// Visualization
    void draw();
    void draw2D(int width, int height);
	QImage fontImage;

	// Synthesis
	void materialize(SurfaceMeshModel * m);
	DynamicVoxel::QuadMesh cached_mesh;

    // Analysis
    Node * rootBySize();
    Node * rootByValence();

	// DEBUG:
	std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
	void printAdjacency();
};

}
