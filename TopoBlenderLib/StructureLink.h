#pragma once

#include "StructureGlobal.h"
#include "NurbsDraw.h"

typedef Array1D_Vector4d LinkCoords;
typedef QPair< QString,Vector4d > NodeCoord;
typedef QPair< QString,LinkCoords > NodeCoords;

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
	std::vector<LinkCoords> coord;
	QString id;
	QString type;
	PropertyMap property;

	bool hasProperty(QString propertyName) { return property.contains(propertyName); }

	void setCoord( QString nodeID, Array1D_Vector4d newCoord );
	void setCoordOther( QString nodeID, Array1D_Vector4d newCoord );

	Array1D_Vector4d getCoord(QString nodeID);
	Array1D_Vector4d getCoordOther(QString nodeID);
	Vector4d getMiddleCoord(QString nodeID);
	void invertCoords( QString nodeID );
	Node * getNode(QString nodeID);
	Node * otherNode(QString nodeID);

	Node * getNodeHasProperty(QString propertyName, QVariant propertyValue);

	// Modify
	void replace(QString oldNodeID, Node *newNode, Array1D_Vector4d newCoord);

	// Constructors
    Link(Node * node1, Node * node2, LinkCoords coord_n1, LinkCoords coord_n2, QString link_type, QString ID);
	Link(){	n1 = n2 = NULL; coord.resize(2); }
	~Link();

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

	/// Helpers:
	static QVector<Link*> haveProperty( QVector<Link*> links, QString propertyName );
	static QVector<Link*> haveProperty( QVector<Link*> links, QString propertyName, QVariant propertyValue );

	Vector3d delta();

	// State
	void pushState();
	void popState();

private:
	QMap< QString, QVariant> state;
};

}

Q_DECLARE_METATYPE( Structure::Link* )
Q_DECLARE_METATYPE( NodeCoord )
Q_DECLARE_METATYPE( NodeCoords )
Q_DECLARE_METATYPE( QVector< NodeCoord > )
