#include <QMouseEvent>
#include "surfacemesh_mode_info.h"
#include "StarlabDrawArea.h"
#include <qgl.h>
using namespace qglviewer;

// Fast OpenGL text
#include "font.inl"

void surfacemesh_mode_info::createEdit()
{
	SurfaceMeshHelper h(mesh());
	faceCenters = h.computeFaceBarycenters();
	faceNormals = h.computeFaceNormals();

	visualize = QVector<bool>(HDGE_IDX+1, true);
	visualize[HDGE_IDX] = false; // Hide half-edges by default
}

void surfacemesh_mode_info::drawWithNames()
{
    int i = 0;

	Vector3VertexProperty points = mesh()->get_vertex_property<Vector3>(VPOINT);

	// Vertices
    foreach(const Vertex vit, mesh()->vertices()){
		Vector3 v = points[vit];
        glPushName(i++);
        glBegin(GL_POINTS);
            glVertex3d(v.x(), v.y(), v.z());
        glEnd();
        glPopName();
    }

	// Faces
	foreach(const Face f, mesh()->faces()){
		glPushName(i++);
		glBegin(GL_POINTS);
		QVector4D v = faceCenters[f];
		glVertex3d(v.x(), v.y(), v.z());
		glEnd();
		glPopName();
	}

	// Edges
	foreach(const Edge e, mesh()->edges()){
		glPushName(i++);
		glBegin(GL_POINTS);
		QVector3D p1 = points[mesh()->vertex(e,0)];
		QVector3D p2 = points[mesh()->vertex(e,1)];
		QVector3D v = (p1 + p2) * 0.5;
		glVertex3d(v.x(), v.y(), v.z());
		glEnd();
		glPopName();
	}
}

void surfacemesh_mode_info::postSelection(const QPoint& p)
{
	int nv = mesh()->n_vertices();
	int nf = mesh()->n_faces();
	int ne = mesh()->n_edges();

	// Find actual index
	int i = drawArea()->selectedName();
	if(i < nv)
		selectedType = 0;
	else if(i < nv + nf){
		selectedType = 1;
		i -= nv;
	}
	else if(i < nv + nf + ne){
		selectedType = 2;
		i -= (nv + nf);
	}

	selectedIdx = i;
}

bool surfacemesh_mode_info::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	if(event->key() == Qt::Key_V) { 
		visualize[VERT_IDX] = !visualize[VERT_IDX]; 
		used = true;
	}

	if(event->key() == Qt::Key_F) { 
		visualize[FACE_IDX] = !visualize[FACE_IDX]; 
		used = true;
	}

	if(event->key() == Qt::Key_E) { 
		visualize[EDGE_IDX] = !visualize[EDGE_IDX]; 
		used = true;
	}

	if(event->key() == Qt::Key_H) { 
		visualize[HDGE_IDX] = !visualize[HDGE_IDX]; 
		used = true;
	}

	drawArea()->updateGL();

	return used; 
}

bool surfacemesh_mode_info::keyReleaseEvent( QKeyEvent* event )
{
	return false;
}

void surfacemesh_mode_info::decorate()
{
	int idx = drawArea()->selectedName();

	if(idx < 0)
	{
		// Draw visible visualizations on entire mesh
		if(visualize[VERT_IDX]) drawIndex(VERT_IDX, QColor(255,0,0));
		if(visualize[FACE_IDX]) drawIndex(FACE_IDX, QColor(0,255,0));
		if(visualize[EDGE_IDX]) drawIndex(EDGE_IDX, QColor(0,0,255));
		if(visualize[HDGE_IDX]) drawIndex(HDGE_IDX, QColor(255,127,0));
	}
	else
	{
		// Draw selected item
		Vector3VertexProperty points = mesh()->get_vertex_property<Vector3>(VPOINT);

		switch(selectedType){
		case 0:
			drawArea()->drawPoint(points[Vertex(selectedIdx)]);
			break;
		case 1:
			break;
		case 2:
			break;
		}
	}
}

void surfacemesh_mode_info::drawIndex(DrawElementType indexType, QColor color, double vt)
{
	char buf[64];
	Vec viewDir = drawArea()->camera()->viewDirection().unit();
	Vector3 cameraNormal(viewDir[0],viewDir[1],viewDir[2]);

	Vector3VertexProperty points = mesh()->get_vertex_property<Vector3>(VPOINT);

	drawArea()->qglColor(color);

	drawArea()->startScreenCoordinatesSystem();

	initFont();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fontTexture);

	glBegin(GL_QUADS);
	switch (indexType)
	{
	case VERT_IDX:
		foreach(const Vertex v, mesh()->vertices())
		{
			Face f = mesh()->face(mesh()->halfedge(v));
			if(dot(faceNormals[f], cameraNormal) > vt) continue;

			QVector3D p = points[v];
			Vec proj = drawArea()->camera()->projectedCoordinatesOf(Vec(p.x(), p.y(), p.z()));
			sprintf(buf,"%d", v.idx());
			drawStringQuad(proj.x - (stringWidth(buf) * 0.5), proj.y, buf);
		}
		break;

	case FACE_IDX:
		foreach(const Face f, mesh()->faces()){
			if(dot(faceNormals[f], cameraNormal) > vt) continue;

			QVector3D c = faceCenters[f];
			Vec proj = drawArea()->camera()->projectedCoordinatesOf(Vec(c.x(), c.y(), c.z()));
			sprintf(buf,"%d", f.idx());
			drawStringQuad(proj.x - (stringWidth(buf) * 0.5), proj.y, buf);
		}
		break;

	case EDGE_IDX:
		foreach(const Edge e, mesh()->edges()){
			Face f = mesh()->face(mesh()->halfedge(e,0));
			if(dot(faceNormals[f], cameraNormal) > vt) continue;

			QVector3D p1 = points[mesh()->vertex(e,0)];
			QVector3D p2 = points[mesh()->vertex(e,1)];
			QVector3D c = (p1 + p2) * 0.5;

			Vec proj = drawArea()->camera()->projectedCoordinatesOf(Vec(c.x(), c.y(), c.z()));
			sprintf(buf,"%d", e.idx());
			drawStringQuad(proj.x - (stringWidth(buf) * 0.5), proj.y, buf);
		}
		break;

	case HDGE_IDX:
		foreach(const Halfedge h, mesh()->halfedges()){
			Face f = mesh()->face(h);
			if(dot(faceNormals[f], cameraNormal) > vt) continue;

			QVector3D p1 = points[mesh()->from_vertex(h)];
			QVector3D p2 = points[mesh()->to_vertex(h)];
			QVector3D c = (p1 + p2 + faceCenters[f]) / 3.0;

			Vec proj = drawArea()->camera()->projectedCoordinatesOf(Vec(c.x(), c.y(), c.z()));
			sprintf(buf,"%d", h.idx());
			drawStringQuad(proj.x - (stringWidth(buf) * 0.5), proj.y, buf);
		}
		break;
	}
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	drawArea()->stopScreenCoordinatesSystem();
}

void surfacemesh_mode_info::drawSelectedItem( DrawElementType, QColor )
{

}

void surfacemesh_mode_info::endSelection( const QPoint& p )
{
	drawArea()->defaultEndSelection(p);
}

Q_EXPORT_PLUGIN(surfacemesh_mode_info)
