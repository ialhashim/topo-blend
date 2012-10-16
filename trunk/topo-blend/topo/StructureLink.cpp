#include "StructureLink.h"
#include "StructureNode.h"

void Structure::Link::draw()
{
	std::vector<Vector3> pos(2, Vector3(0));

	Vec3d c1(coord[0].x(), coord[0].y(), 0);
	Vec3d c2(coord[1].x(), coord[1].y(), 0);

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
