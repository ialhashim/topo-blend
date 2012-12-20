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
    QMap< QString, QVariant > link_property;

	Vec4d coord[2];

	void setCoord(QString nodeID, Vec4d newCoord);
	Vec4d getCoord(QString nodeID);
	Vec4d getCoordOther(QString nodeID);
	Node * getNode(QString nodeID);
	Node * otherNode(QString nodeID);

	// Constructors
    Link(Node * node1, Node * node2, Vec4d coord_n1, Vec4d coord_n2, QString link_type, QString ID)
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
	bool hasNodeProperty(QString propertyName, QVariant propertyValue);
	Vector3 position(QString nodeID);

	// Visualization
    void draw();

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }
};

}
