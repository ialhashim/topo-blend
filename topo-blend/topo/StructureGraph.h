#pragma once

#include "DynamicVoxel.h"

#include "StructureCurve.h"
#include "StructureSheet.h"

namespace Structure{

struct Graph
{
	// Properties
    QSet<Node*> nodes;
    QSet<Link> edges;
    QBox3D bbox();
    QMap< QString, QVariant > property;
	
	// Constructors
	Graph();
    ~Graph();
	
	// Modifiers
    Node * addNode( Node * n );
    Link addEdge( Node * n1, Node * n2 );

    QString linkName( Node * n1, Node * n2 );

    // Accessors
    Node* getNode(QString nodeID);
	Vector3 nodeIntersection( Node * n1, Node * n2 );
	
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

	// DEBUG:
	std::vector<Vector3> debugPoints, debugPoints2, debugPoints3;
};

}
