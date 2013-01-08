#pragma once

#include <float.h>
#include <qgl.h>
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

        glEnable(GL_LIGHTING);

        drawTris();
        drawQuads();

        glDisable(GL_LIGHTING);

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

    size_t count(){
        return points.size();
    }
};

class VectorSoup : public RenderObject::Base{
    QVector< QPair<QVector3D,QVector3D> > vectors;
    QVector< double > vectorLengths;
    double maxLen;
public:
    VectorSoup(const QColor& c = Qt::green):RenderObject::Base(1, c){ maxLen = -DBL_MAX; }

    void addVector(const QVector3D& start, const QVector3D& direction){
        vectors.push_back( qMakePair(start,direction) );
        double l = direction.length();
        vectorLengths.push_back(l);
        maxLen = qMax(l,maxLen);
    }

    virtual void draw(){
        glDisable(GL_LIGHTING);
        glLineWidth(1);
        glBegin(GL_LINES);
        for(int i = 0; i < (int) vectors.size(); i++){
            // Color
            double d = vectorLengths[i] / maxLen;
            QColor c( _color.red() * d, _color.green() * d, _color.blue() * d );
            glColorQt(c);

            // Line
            glVertQt(vectors[i].first);
            glVertQt((vectors[i].first + vectors[i].second));
        }
        glEnd();

        glPointSize(3);
        glBegin(GL_POINTS);
        for(int i = 0; i < (int) vectors.size(); i++){
            // Color
            double d = vectorLengths[i] / maxLen;
            QColor c( _color.red() * d, _color.green() * d, _color.blue() * d );
            glColorQt(c);

            // Point
            glVertQt((vectors[i].first + vectors[i].second));
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
};


class PlaneSoup : public RenderObject::Base{
	QVector< QPair<QVector3D,QVector3D> > planes;
	double scale;
public:
	PlaneSoup(double s = 1.0, const QColor& c = Qt::green):RenderObject::Base(1, c)
	{ scale = s; }

	void addPlane(const QVector3D& center, const QVector3D& normal){
		planes.push_back( qMakePair(center, normal) );
	}

	virtual void draw(){
		glDisable(GL_LIGHTING);

		// Rendering options
		glEnable (GL_POINT_SMOOTH);
		glEnable (GL_LINE_SMOOTH);
		glEnable (GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);

		float color[4];
		glGetFloatv(GL_CURRENT_COLOR, color);

		// Bake geometry
		int c = planes.size();
		std::vector<Vec3d> n(c),u(c),v(c),p1(c),p2(c),p3(c),p4(c);
		for(int i = 0; i < c; i++){
			n[i] = planes[i].second;
			u[i] = scale * orthogonalVector(n[i]).normalized();
			v[i] = scale * cross(n[i], u[i]).normalized();

			p1[i] = planes[i].first + (u[i] + v[i]);
			p2[i] = planes[i].first + (-u[i] + v[i]);
			p3[i] = planes[i].first + (-v[i] + u[i]);
			p4[i] = planes[i].first + (-v[i] + -u[i]);
		}

		// Draw Borders
		glColor4f(color[0]*0.8, color[1]*0.8, color[2]*0.8, 0.5);
		glLineWidth(2.0);
		glPolygonMode(GL_FRONT,GL_LINE);
		glBegin(GL_QUADS);
		for(int i = 0; i < c; i++)
		{
			glVertex3dv(p1[i]);
			glVertex3dv(p2[i]);
			glVertex3dv(p4[i]);
			glVertex3dv(p3[i]);
		}
		glEnd();

		// Draw Center
		glColor3f(color[0], color[1], color[2]);
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for(int i = 0; i < c; i++) glVertex3dv(Vec3d(planes[i].first));
		glEnd();
		glPointSize(8.0);
		glColor4f(1, 1, 1, 0.5);
		glBegin(GL_POINTS);
		for(int i = 0; i < c; i++) glVertex3dv(Vec3d(planes[i].first));
		glEnd();

		// Draw Normal
		glBegin(GL_LINES);
		for(int i = 0; i < c; i++)
		{
			Vec3d center = Vec3d(planes[i].first);
			glVertex3dv(center);
			glVertex3dv(center + (n[i] * 0.2 * scale));
		}
		glEnd();

		// Draw Transparent Fills
		glColor4f(color[0], color[1], color[2], 0.05f);
		glPolygonMode(GL_FRONT,GL_FILL);
		glBegin(GL_QUADS);
		for(int i = 0; i < c; i++)
		{
			glVertex3dv(p1[i]);
			glVertex3dv(p2[i]);
			glVertex3dv(p4[i]);
			glVertex3dv(p3[i]);
		}
		glEnd();

		glDisable(GL_BLEND);
		glEnable(GL_LIGHTING);
	}

	static Vec3d orthogonalVector(const Vec3d& n) {
		if ((abs(n.y()) >= 0.9 * abs(n.x())) &&
			abs(n.z()) >= 0.9 * abs(n.x())) return Vec3d(0.0, -n.z(), n.y());
		else if ( abs(n.x()) >= 0.9 * abs(n.y()) &&
			abs(n.z()) >= 0.9 * abs(n.y()) ) return Vec3d(-n.z(), 0.0, n.x());
		else return Vec3d(-n.y(), n.x(), 0.0);
	}
};

static QColor qtColdColor(double value, double min = 0.0, double max = 1.0){
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

static QColor qtJetColorMap(double value, double min = 0.0, double max = 1.0)
{
    unsigned char rgb[3];
    unsigned char c1=144;
    float max4=(max-min)/4;
    value-=min;
    if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
    else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
    else if(value<max4)
    {rgb[0]=0;rgb[1]=0;rgb[2]=c1+(unsigned char)((255-c1)*value/max4);}
    else if(value<2*max4)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max4)/max4);rgb[2]=255;}
    else if(value<3*max4)
    {rgb[0]=(unsigned char)(255*(value-2*max4)/max4);rgb[1]=255;rgb[2]=255-rgb[0];}
    else if(value<max)
    {rgb[0]=255;rgb[1]=(unsigned char)(255-255*(value-3*max4)/max4);rgb[2]=0;}
    else {rgb[0]=255;rgb[1]=rgb[2]=0;}
    return QColor(rgb[0],rgb[1],rgb[2]);
}

#include <time.h>
static std::vector<double> randomColor()
{
    std::vector<double> color;

    float r = ((rand() % 225) + 30) / 255.0f;
    float g = ((rand() % 230) + 25) / 255.0f;
    float b = ((rand() % 235) + 20) / 255.0f;

    color.push_back(r);
    color.push_back(g);
    color.push_back(b);
    color.push_back(1.0);

    return color;
}

static std::vector< std::vector<double> > randomColors( int count )
{
	srand(time(NULL));

	std::vector< std::vector<double> > colors(count);
    for (int i = 0; i < count; i++)
        colors[i] = randomColor();
	return colors;
}

static QColor qRandomColor()
{
    std::vector<double> c = randomColor();
    return QColor::fromRgbF( c[0], c[1], c[2], c[3] );
}
