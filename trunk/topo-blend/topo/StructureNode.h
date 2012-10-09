#pragma once
#include "StructureLink.h"
#include "SurfaceMeshHelper.h"

namespace Structure{

struct Node
{
	// Properties
	QString id;
    virtual QString type() = 0;
    virtual QBox3D bbox() = 0;
    QMap< QString, QVariant > property;

	// Connections
    QSet< Link* > edges;
	int valence() { return edges.size(); }
	bool disconnected() { return valence() == 0; }
	
    // Visualization
    virtual void draw() = 0;
    virtual void draw2D() = 0;
    QMap< QString, QVariant > vis_property;
};

}
