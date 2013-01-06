#pragma once

#include "StructureGlobal.h"

typedef Array1D_Vec4d LinkCoords;

namespace Structure{

struct Node;

static QString CURVE = "CURVE";
static QString SHEET = "SHEET";
static QString POINT_EDGE = "POINT";
static QString LINE_EDGE = "LINE";

struct Link
{
	// Properties
	Node *n1, *n2;	
	QString id;
	QString type;
    QMap< QString, QVariant > property;

	LinkCoords coord[2];

	void Link::setCoord( QString nodeID, std::vector<Vec4d> newCoord );

	std::vector<Vec4d> getCoord(QString nodeID);
	std::vector<Vec4d> getCoordOther(QString nodeID);
	Vec4d getMiddleCoord(QString nodeID);
	Node * getNode(QString nodeID);
	Node * otherNode(QString nodeID);

	// Constructors
    Link(Node * node1, Node * node2, LinkCoords coord_n1, LinkCoords coord_n2, QString link_type, QString ID)
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
	Vector3 positionOther(QString nodeID);

	// Visualization
    void draw();

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }
};

}

Q_DECLARE_METATYPE(Structure::Link*)
