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

	Vector3 coord[2];
	
	// Constructors
    Link(Node * node1, Node * node2, Vector3 coord_n1, Vector3 coord_n2, QString link_type, QString ID)
	{
		assert(n1 != n2);
	
		this->n1 = node1;
		this->n2 = node2;
		this->type = link_type;
        this->id = ID;

		this->coord[0] = coord_n1;
		this->coord[1] = coord_n2;
	}
	
	// Visualization
    void draw();

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }
};

}

static inline uint qHash(const Structure::Link& edge) { return qHash(edge.id); }
