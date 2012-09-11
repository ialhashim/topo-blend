#pragma once

#include "RenderObject.h"

#define glVertQt(v) glVertex3d(v.x(), v.y(), v.z())
#define glColorQt(c) glColor4d(c.redF(), c.greenF(), c.blueF(), c.alphaF())

class PolygonSoup : public RenderObject::Base{
	QVector< QVector<QVector3D> > polys;
	QVector< QVector3D > polys_normals;
	QVector< QColor > polys_colors;

public:
	PolygonSoup():RenderObject::Base(1, Qt::black){}

	virtual void draw(){
		glDisable(GL_LIGHTING);

		drawTris();
		drawQuads();

		// Draw borders as lines and points to force drawing something
		glLineWidth(2.0f);
		glColor4d(0,0,0,1);
		glBegin(GL_LINES);
		foreach(QVector<QVector3D> poly, polys){
			for(int i = 0; i < (int) poly.size(); i++){
				glVertQt(poly[i]);
				glVertQt(poly[(i+1) % poly.size()]);
			}
		}
		glEnd();

		glPointSize(3.0f);
		glBegin(GL_POINTS);
		foreach(QVector<QVector3D> poly, polys){
			for(int i = 0; i < (int) poly.size(); i++)
				glVertQt(poly[i]);
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}

	void drawTris(bool isColored = true){
		glBegin(GL_TRIANGLES);
		for(int i = 0; i < (int) polys.size(); i++)
		{
			if(polys[i].size() != 3) continue;
			else{
				glNormal3d(polys_normals[i].x(),polys_normals[i].y(),polys_normals[i].z());
				if(isColored) glColorQt(polys_colors[i]);
				for(int p = 0; p < 3; p++) glVertQt(polys[i][p]);
			}
		}
		glEnd();
	}

	void drawQuads(bool isColored = true){
		glBegin(GL_QUADS);
		for(int i = 0; i < (int) polys.size(); i++)
		{
			if(polys[i].size() != 4) continue;
			else{
				glNormal3d(polys_normals[i].x(),polys_normals[i].y(),polys_normals[i].z());
				if(isColored) glColorQt(polys_colors[i]);
				for(int p = 0; p < 4; p++) glVertQt(polys[i][p]);
			}
		}
		glEnd();
	}

	void addPoly(const QVector<QVector3D>& points, const QColor& c = Qt::red){
		if(points.size() < 3) return;
		else
			polys.push_back(points);

		// Compute normal from 3 points
		polys_normals.push_back(QVector3D::crossProduct(points[1] - points[0], points[2] - points[0]).normalized());
		polys_colors.push_back(c);
	}
};

class LineSegments : public RenderObject::Base{
	QVector< QPair<QVector3D,QVector3D> > lines;
	QVector< QColor > lines_colors;
public:
	LineSegments():RenderObject::Base(1, Qt::black){}

	virtual void draw(){
		glDisable(GL_LIGHTING);

		glLineWidth(3);
		glBegin(GL_LINES);
		for(int i = 0; i < (int) lines.size(); i++){
			glColorQt(lines_colors[i]);
			glVertQt(lines[i].first);
			glVertQt(lines[i].second);
		}
		glEnd();

		glPointSize(6);
		glBegin(GL_POINTS);
		for(int i = 0; i < (int) lines.size(); i++){
			glColorQt(lines_colors[i]);
			glVertQt(lines[i].first);
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}

	void addLine(const QVector3D& p1, const QVector3D& p2, const QColor& c = Qt::blue){
		lines.push_back( qMakePair(p1,p2) );
		lines_colors.push_back(c);
	}
};

class PointSoup : public RenderObject::Base{
	QVector< QVector3D > points;
	QVector< QColor > points_colors;
public:
	PointSoup():RenderObject::Base(1, Qt::black){}

	virtual void draw(){
		glDisable(GL_LIGHTING);

		glPointSize(6);
		glBegin(GL_POINTS);
		for(int i = 0; i < (int) points.size(); i++){
			glColorQt(points_colors[i]);
			glVertQt(points[i]);
		}
		glEnd();

		glEnable(GL_LIGHTING);
	}

	void addPoint(const QVector3D& p, const QColor& c = Qt::blue){
		points.push_back(p);
		points_colors.push_back(c);
	}
};

QColor qtColdColor(double value, double min = 0.0, double max = 1.0){
	unsigned char rgb[3];
	value-=min;
	if(value==HUGE_VAL)
	{rgb[0]=rgb[1]=rgb[2]=255;}
	else if(value<0)
	{rgb[0]=rgb[1]=rgb[2]=0;}
	else if(value<max)
	{rgb[0]=0;rgb[1]=0;rgb[2]=(unsigned char)(255*value/max);}
	else {rgb[0]=rgb[1]=0;rgb[2]=255;}
	return QColor(rgb[0],rgb[1],rgb[2]);
}
