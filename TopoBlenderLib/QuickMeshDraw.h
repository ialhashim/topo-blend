#pragma once

#include <qgl.h>
#include "SurfaceMeshModel.h"

struct QuickMeshDraw{
	static void drawMeshWireFrame( SurfaceMeshModel * mesh )
	{
		Surface_mesh::Face_iterator fit, fend=mesh->faces_end();
		Surface_mesh::Vertex_around_face_circulator fvit, fvend;
		Surface_mesh::Vertex_property<Vector3> points = mesh->vertex_property<Vector3>("v:point");
		Surface_mesh::Face_property<Vector3> fnormals = mesh->face_property<Vector3>("f:normal");

		glEnable (GL_BLEND);
		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glColor4d(0,1,1, 0.25);
		glLineWidth(1.0f);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);

		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		for (fit=mesh->faces_begin(); fit!=fend; ++fit){
			glBegin(GL_POLYGON);
			glNormal3( fnormals[fit] );
			fvit = fvend = mesh->vertices(fit);
			do{ glVector3( points[fvit] ); } while (++fvit != fvend);
			glEnd();
		}
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

		glDisable(GL_CULL_FACE);
	}
};
