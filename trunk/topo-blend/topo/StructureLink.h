#pragma once

#include <assert.h>
#include <QVariant>
#include <QMap>
#include <QSet>

#include "NurbsDraw.h"
#include "../CustomDrawObjects.h"

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
	
	// Constructors
    Link(Node * node1, Node * node2, QString link_type, QString ID)
	{
		assert(n1 != n2);
	
		this->n1 = node1;
		this->n2 = node2;
		this->type = link_type;
        this->id = ID;
	}
	
	// Visualization
    void draw()
    {

    }

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }
};

}

static inline uint qHash(const Structure::Link& edge) { return qHash(edge.id); }
