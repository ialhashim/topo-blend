#pragma once

#include "StructureCurve.h"
#include "StructureSheet.h"

namespace Structure{

struct Graph
{
	// Properties
    QSet<Node*> nodes;
    QSet<Link> edges;
    QBox3D bbox();
	
	// Constructors
	Graph(){}
    ~Graph();
	
	// Modifiers
    Node * addNode( Node * n );
    Link addEdge( Node * n1, Node * n2 );

    QString linkName( Node * n1, Node * n2 );

    // Accessors
    Node* getNode(QString nodeID);
	
	// Input / Output
	void saveToFile(QString fileName);
	void loadFromFile(QString fileName);

	// Visualization
    void draw() const;
    void draw2D() const;
};

}
