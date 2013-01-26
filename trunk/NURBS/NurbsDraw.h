#pragma once
#include <qgl.h>

#include "NURBSCurve.h"
#include "NURBSRectangle.h"

#define glVector3( v ) glVertex3d( v.x(), v.y(), v.z() )
#define glNormal3( v ) glNormal3d( v.x(), v.y(), v.z() )

typedef std::pair<Vector3,Vector3> PairVector3;
#define glLine(v1,v2) glVector3(v1);glVector3(v2)

namespace NURBS
{

class CurveDraw{
public:
    static void draw( NURBSCurved * nc, QColor curve_color = QColor(0,255,255), bool drawControl = false)
    {
        if(drawControl)
        {
			glEnable( GL_POINT_SMOOTH );

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
        }

        // Draw actual curve
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glColor4d(curve_color.redF(),curve_color.greenF(),curve_color.blueF(),curve_color.alphaF());
        glLineWidth (4.0f);

        glBegin(GL_LINE_STRIP);
        int nSteps = 60;
        for(int i = 0; i <= nSteps; i++)
        {
            double u = double(i) / nSteps;

            Vec3d p( nc->GetPosition( u ) );
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();

		// Draw start indicator
		glColor3d(1,0,0);
		glBegin(GL_LINES);
		glVector3(nc->GetPosition( 0 )); glVector3(nc->GetPosition( 0.01 ));
		glEnd();

        glEnable(GL_LIGHTING);
    }
};

class SurfaceDraw{
public:
    static void draw( NURBSRectangled * nc, QColor sheet_color = QColor(0,255,255), bool drawControl = false )
	{
		glEnable( GL_POINT_SMOOTH );

		int width = nc->GetNumCtrlPoints(0);
		int length = nc->GetNumCtrlPoints(1);

        if(drawControl)
        {
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

		glDisable(GL_LIGHTING);


		// Wireframe
		glColor3d(0,0,0);
		glLineWidth(1.0);
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


		// Draw UV indicator
		if(!nc->quads.empty())
		{
			SurfaceQuad quad = nc->quads.front();

			glLineWidth(8);

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
};

}
