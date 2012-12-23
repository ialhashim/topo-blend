#include "StructureLink.h"
#include "StructureNode.h"

using namespace Structure;

void Link::setCoord( QString nodeID, Vec4d newCoord )
{
	if(n1->id == nodeID) coord[0] = std::vector<Vec4d> (1, newCoord);
	if(n2->id == nodeID) coord[1] = std::vector<Vec4d> (1, newCoord);
}

Vec4d Link::getCoord( QString nodeID )
{
	if(n1->id == nodeID) return coord[0].front();
	if(n2->id == nodeID) return coord[1].front();
	return Vec4d(DBL_MAX);
}

Vec4d Structure::Link::getCoordOther( QString nodeID )
{
	return getCoord(otherNode(nodeID)->id);
}

Node * Link::otherNode( QString nodeID )
{
	if(n1->id == nodeID) return n2;
	else return n1;
}

Node * Link::getNode( QString nodeID )
{
	if(n1->id == nodeID) return n1;
	else return n2;
}

void Link::draw()
{
	std::vector<Vector3> pos(2, Vector3(0));

	Vec4d c1 = coord[0].front();
	Vec4d c2 = coord[1].front();

	n1->get(c1, pos[0], noFrame());
	n2->get(c2, pos[1], noFrame());

	glDisable( GL_LIGHTING );
	glEnable( GL_POINT_SMOOTH );

	glPointSize(10.0f);
	glColor3d(0,0,1);glBegin(GL_POINTS);glVector3(pos[0]);glVector3(pos[1]);glEnd();

	glPointSize(12.0f);
	glColor3d(1,1,1);glBegin(GL_POINTS);glVector3(pos[0]);glVector3(pos[1]);glEnd();

	glEnable(GL_LIGHTING);
}

bool Link::hasNode( QString nodeID )
{
	return n1->id == nodeID || n2->id == nodeID;
}

bool Link::hasNodeProperty( QString propertyName, QVariant propertyValue )
{
	bool pn1 = n1->hasProperty(propertyName) && n1->property[propertyName] == propertyValue;
	bool pn2 = n2->hasProperty(propertyName) && n2->property[propertyName] == propertyValue;

	return pn1 || pn2;
}

SurfaceMeshTypes::Vector3 Link::position( QString nodeID )
{
	Node * n = n1->id == nodeID ? n1 : n2;
	assert(n->id == nodeID);

	Vector3 pos(0);
	n->get(getCoord(nodeID),pos);
	return pos;
}
