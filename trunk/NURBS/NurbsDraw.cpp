#include <QtOpenGL>
#include "NurbsDraw.h"

using namespace NURBS;

void CurveDraw::draw( NURBSCurved * nc, QColor curve_color, bool drawControl, double scaling)
{
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);

    if(drawControl)
    {
        glEnable( GL_POINT_SMOOTH );

        // Draw control points
        glPointSize(6.0f * scaling);
        glColor3d(1,1,1);
        glBegin(GL_POINTS);
        foreach(Vector3d p, nc->getControlPoints())
            glVertex3d(p.x(), p.y(), p.z());
        glEnd();

        // Draw connections of control points
        glLineWidth(2.0f * scaling);
        glColor3d(0,0,0);
        glBegin(GL_LINE_STRIP);
        foreach(Vector3d p, nc->getControlPoints())
            glVertex3d(p.x(), p.y(), p.z());
        glEnd();
    }

    // Draw actual curve
    glColor4d(curve_color.redF(),curve_color.greenF(),curve_color.blueF(),curve_color.alphaF());

    Array1D_Vector3 points;

    // Evaluate
    int nSteps = 60;
    for(int i = 0; i <= nSteps; i++){
        double u = double(i) / nSteps;
        Vector3d p( nc->GetPosition( u ) );
        points.push_back(p);
    }

    // Draw start indicator (thick portion)
    glLineWidth (5.0f * scaling);
    glBegin(GL_LINE_STRIP);
    for(int i = 0; i <= nSteps; i++){
        double u = double(i) / nSteps;
        if(u > 0.2) break;
        Vector3d p( nc->GetPosition( u ) );
        glVertex3d(p.x(), p.y(), p.z());
    }
    glEnd();

    glLineWidth (2.0f * scaling);
    glBegin(GL_LINE_STRIP);
    foreach(Vector3d p, points) glVertex3d(p.x(), p.y(), p.z());
    glEnd();

    glEnable(GL_LIGHTING);
}

void SurfaceDraw::draw( NURBSRectangled * nc, QColor sheet_color, bool drawControl, double scaling, QColor wireframe_color)
{
    glEnable( GL_POINT_SMOOTH );

    int width = nc->GetNumCtrlPoints(0);
    int length = nc->GetNumCtrlPoints(1);

    if(drawControl)
    {
        // Draw control points
        glDisable(GL_LIGHTING);
        glPointSize(6.0f * scaling);
        glColor3d(1,1,1);
        glBegin(GL_POINTS);
        for(int i = 0; i < width; i++)
        {
            for(int j = 0; j < length; j++)
            {
                Vector3 p = nc->GetControlPoint(i,j);
                glVertex3d(p.x(), p.y(), p.z());
            }
        }
        glEnd();
        glEnable(GL_LIGHTING);

        // Draw connections of control points
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f * scaling);
        glColor3d(0,0,0);

        // Draw "horizontal"
        for(int i = 0; i < width; i++)
        {
            glBegin(GL_LINE_STRIP);
            for(int j = 0; j < length; j++)
            {
                Vector3 p = nc->GetControlPoint(i,j);
                glVertex3d(p.x(), p.y(), p.z());
            }
            glEnd();
        }

        // Draw verticals
        for(int j = 0; j < length; j++)
        {
            glBegin(GL_LINE_STRIP);
            for(int i = 0; i < width; i++)
            {
                Vector3 p = nc->GetControlPoint(i,j);
                glVertex3d(p.x(), p.y(), p.z());
            }
            glEnd();
        }

        // Draw surface of control cage
        /*std::vector< std::vector<Vector3> > tris = nc->triangulateControlCage();
        glBegin(GL_TRIANGLES);
        for(int i = 0; i < (int)tris.size(); i++){
            std::vector<Vector3> & tri = tris[i];
            (i % 2 == 0) ? glColor3d(0,1,0) : glColor3d(0,0,1);
            glVector3(tri[0]);
            glVector3(tri[1]);
            glVector3(tri[2]);
        }
        glEnd();*/
    }

    // Draw actual surface
    if(nc->quads.empty())
    {
        double resolution = (nc->mCtrlPoint.front().front() - nc->mCtrlPoint.back().back()).norm() * 0.05;
        nc->generateSurfaceQuads( resolution );
    }

    glEnable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glColor4d(sheet_color.redF(),sheet_color.greenF(),sheet_color.blueF(),sheet_color.alphaF());

    glBegin(GL_QUADS);
    foreach(SurfaceQuad quad, nc->quads){
        for(int i = 0; i < 4; i++){
            glNormal3(quad.n[i]);
            glVector3(quad.p[i]);
        }
    }
    glEnd();

	// Wireframe
	glDisable(GL_LIGHTING);
	glColor4d(wireframe_color.redF(),wireframe_color.greenF(),wireframe_color.blueF(),wireframe_color.alphaF());
	glLineWidth(1.0 * scaling);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBegin(GL_QUADS);
	foreach(SurfaceQuad quad, nc->quads){
		for(int i = 0; i < 4; i++){
			glNormal3(quad.n[i]);
			glVector3(quad.p[i]);
		}
	}
	glEnd();
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_LIGHTING);

    // Draw UV indicator
    if(!nc->quads.empty())
    {
        SurfaceQuad quad = nc->quads.front();

        glLineWidth(3);

        glColor3d(1,0,0);
        glBegin(GL_LINES);
        glVector3(quad.p[0]); glVector3(quad.p[1]);
        glEnd();

        glColor3d(0,1,0);
        glBegin(GL_LINES);
        glVector3(quad.p[0]); glVector3(quad.p[3]);
        glEnd();
    }

    glEnable(GL_LIGHTING);
}
