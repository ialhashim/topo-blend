#pragma once

#include "StructureGlobal.h"

namespace Structure{

struct Node;

static QString CURVE = "CURVE";
static QString SHEET = "SHEET";

struct Link
{
	// Properties
	Node *n1, *n2;	
	QString id;
	QString type;
    QMap< QString, QVariant > property;

	Vec2d coord[2];

	void setCoord(QString nodeID, Vec2d newCoord);
	Vec2d getCoord(QString nodeID);
	Node * otherNode(QString nodeID);

	// Constructors
    Link(Node * node1, Node * node2, Vec2d coord_n1, Vec2d coord_n2, QString link_type, QString ID)
	{
		this->n1 = node1;
		this->n2 = node2;
		this->type = link_type;
        this->id = ID;

		this->coord[0] = coord_n1;
		this->coord[1] = coord_n2;
	}
	Link(){	n1 = n2 = NULL; }
	
	// Accessors
	bool hasNode(QString nodeID);

	// Visualization
    void draw();

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }
};

}
