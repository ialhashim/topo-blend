#pragma once

#include "NURBSCurve3.h"
#include "NURBSRectangle.h"

class NurbsCurveDraw{
public:
    static void draw( NURBSCurve3d * nc )
    {
        // Draw control points
        glDisable(GL_LIGHTING);
        glPointSize(6.0f);
        glColor3d(1,1,1);
        glBegin(GL_POINTS);
        foreach(Vec3d p, nc->getControlPoints())
            glVertex3d(p.x(), p.y(), p.z());
        glEnd();
        glEnable(GL_LIGHTING);

        // Draw connections of control points
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        glColor3d(0,0,0);
        glBegin(GL_LINE_STRIP);
        foreach(Vec3d p, nc->getControlPoints())
            glVertex3d(p.x(), p.y(), p.z());
        glEnd();
        glEnable(GL_LIGHTING);


        // Draw actual curve
        glDisable(GL_LIGHTING);
        glLineWidth(4.0f);
        glColor3d(0,1,1);
        int nSteps = 60;
        glBegin(GL_LINE_STRIP);
        for(int i = 0; i <= nSteps; i++)
        {
            double u = double(i) / nSteps;

            Vec3d p( nc->GetPosition( u ) );
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
};

#define glVector3( v ) glVertex3d( v.x(), v.y(), v.z() )
#define glNormal3( v ) glNormal3d( v.x(), v.y(), v.z() )

class NurbsSurfaceDraw{
public:
	static void draw( NURBSRectangled * nc )
	{
		int width = nc->GetNumCtrlPoints(0);
		int length = nc->GetNumCtrlPoints(1);

		// Draw control points
		glDisable(GL_LIGHTING);
		glPointSize(6.0f);
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
		glLineWidth(2.0f);
		glColor3d(0,0,0);

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

		// Draw actual surface
		if(nc->quads.empty()) nc->generateSurfaceQuads(60);

		glEnable(GL_LIGHTING);
		glColor3d(0,1,1);

		glBegin(GL_QUADS);
		foreach(SurfaceQuad quad, nc->quads)
		{
			for(int i = 0; i < 4; i++)
			{
				glNormal3(quad.n[i]);	
				glVector3(quad.p[i]);
			}
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}
};
