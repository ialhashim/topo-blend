#include "StructureLink.h"
#include "StructureNode.h"

using namespace Structure;

void Link::setCoord( QString nodeID, std::vector<Vec4d> newCoord )
{
	if(n1->id == nodeID) coord[0] = newCoord;
	if(n2->id == nodeID) coord[1] = newCoord;
}

std::vector<Vec4d> Link::getCoord( QString nodeID )
{
	if(n1->id == nodeID) return coord[0];
	if(n2->id == nodeID) return coord[1];
	return std::vector<Vec4d>();
}

std::vector<Vec4d> Structure::Link::getCoordOther( QString nodeID )
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
	glDisable( GL_LIGHTING );
	glEnable( GL_POINT_SMOOTH );

	std::vector<Vector3> linkPos;

	for(int j = 0; j < (int)coord[0].size(); j++)
	{
		Vector3 p1(0), p2(0);

		n1->get(coord[0][j], p1);
		n2->get(coord[1][j], p2);

		linkPos.push_back(p1);
		linkPos.push_back(p2);
	}

	for(int i = 0; i < (int)linkPos.size(); i++)
	{
		// Blue
		glPointSize(10.0f);
		glColor3d(0,0,1);glBegin(GL_POINTS);glVector3(linkPos[i]);glEnd();

		// White
		glPointSize(12.0f);
		glColor3d(1,1,1);glBegin(GL_POINTS);glVector3(linkPos[i]);glEnd();
	}

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
	n->get(getCoord(nodeID).front(),pos);
	return pos;
}
