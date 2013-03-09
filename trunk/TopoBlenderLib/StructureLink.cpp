#include "StructureLink.h"
#include "StructureNode.h"

using namespace Structure;

Link::Link( Node * node1, Node * node2, LinkCoords coord_n1, LinkCoords coord_n2, QString link_type, QString ID )
{
	this->n1 = node1;
	this->n2 = node2;
	this->type = link_type;
	this->id = ID;

	this->coord.resize(2);
	this->coord[0] = coord_n1;
	this->coord[1] = coord_n2;
}

void Link::setCoord( QString nodeID, std::vector<Vec4d> newCoord )
{
	if(n1->id == nodeID) coord[0] = newCoord;
	if(n2->id == nodeID) coord[1] = newCoord;
}

void Link::setCoordOther( QString nodeID, std::vector<Vec4d> newCoord )
{
	if(n1->id == nodeID) coord[1] = newCoord;
	if(n2->id == nodeID) coord[0] = newCoord;
}

std::vector<Vec4d> Link::getCoord( QString nodeID )
{
	if(n1->id == nodeID) return coord[0];
	if(n2->id == nodeID) return coord[1];
	return std::vector<Vec4d>();
}

std::vector<Vec4d> Link::getCoordOther( QString nodeID )
{
	return getCoord(otherNode(nodeID)->id);
}

Vec4d Link::getMiddleCoord( QString nodeID )
{
	std::vector<Vec4d> nodeCoords = getCoord(nodeID);
	return nodeCoords[ nodeCoords.size() / 2 ];
}

void Link::replace(QString oldNodeID, Node *newNode, std::vector<Vec4d> newCoord)
{
	if(n1->id == oldNodeID){
		n1 = newNode;
		coord[0] = newCoord;
	}

	if(n2->id == oldNodeID){
		n2 = newNode;
		coord[1] = newCoord;
	}

	// Change my ID
	QString oldID = id;
	id = id.replace( oldNodeID, newNode->id );

	if(id != oldID) qDebug() << QString("Link replace [%1] to [%2]").arg(oldID, id);
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
	std::vector<Node*> ns;

	for(int j = 0; j < (int)coord[0].size(); j++)
	{
		Vector3 p1(0), p2(0);

        std::vector<Vector3> nf = noFrame();

        n1->get(coord[0][j], p1, nf);
        n2->get(coord[1][j], p2, nf);

		linkPos.push_back(p1);
		linkPos.push_back(p2);

		ns.push_back(n1);
		ns.push_back(n2);
	}

	for(int i = 0; i < (int)linkPos.size(); i++)
	{
		// Blue
		glPointSize(3);
		if(ns[i]->type() == CURVE) 
			glColor3d(0,0,1);
		else
			glColor3d(0,1,1);
		glBegin(GL_POINTS);glVector3(linkPos[i]);glEnd();

		// White
		glPointSize(5);
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

Node * Link::getNodeHasProperty( QString propertyName, QVariant propertyValue )
{
	bool pn1 = n1->hasProperty(propertyName) && n1->property[propertyName] == propertyValue;
	bool pn2 = n2->hasProperty(propertyName) && n2->property[propertyName] == propertyValue;

	if(pn1) return n1;
	if(pn2) return n2;
	return NULL;
}

SurfaceMesh::Vector3 Link::position( QString nodeID )
{
	Node * n = n1->id == nodeID ? n1 : n2;
	assert(n->id == nodeID);

	Vector3 pos(0);

    std::vector<Vector3> nf = noFrame();

    n->get( getMiddleCoord(nodeID), pos, nf);
	return pos;
}

SurfaceMesh::Vector3 Link::positionOther( QString nodeID )
{
	return position(otherNode(nodeID)->id);
}

QVector<Link*> Link::haveProperty( QVector<Link*> links, QString propertyName )
{
	QVector<Link*> result;
	foreach(Link* l, links){
		if(l->hasProperty(propertyName))
			result.push_back(l);
	}
	return result;
}

QVector<Link*> Link::haveProperty( QVector<Link*> links, QString propertyName, QVariant propertyValue )
{
	QVector<Link*> result;
	QVector<Link*> linksWithProp = Link::haveProperty(links, propertyName);
	foreach(Link* l, links){
		if(l->property[propertyName] == propertyValue)
			result.push_back(l);
	}
	return result;
}
