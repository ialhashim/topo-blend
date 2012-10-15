#pragma once
#include "StructureGlobal.h"
#include "font.inl"

#define glColorQt(c) glColor4d(c.redF(), c.greenF(), c.blueF(), c.alphaF())

static void drawRect(Vec2i center, int width, int height)
{
	int half_width = width * 0.5;
	int half_height = height * 0.5;
	int x = center.x();
	int y = center.y();

	int x1 = x - half_width;
	int y1 = y - half_height;

	int x2 = x + half_width;
	int y2 = y + half_height;

	glBegin(GL_QUADS);
	glVertex2i(x1, y1); glVertex2i(x2, y1); glVertex2i(x2, y2); glVertex2i(x1, y2);
	glEnd();
}

static void drawLine(Vec2i from, Vec2i to, float thickness = 5)
{
	glLineWidth(thickness);

	glBegin(GL_LINES);
	glVertex2f(from.x(), from.y()); glVertex2f(to.x(), to.y());
	glEnd();
}

static void drawPoint(Vec2i p, float pointSize = 5)
{
	glPointSize(pointSize);
	glBegin(GL_POINTS);
	glVertex2f(p.x(), p.y());
	glEnd();
}

static void drawCirlce(Vec2i center, Scalar radius, bool isFilled = true, int resolution = 10)
{
	int x = center.x();
	int y = center.y();

	if(isFilled)
	{
		glBegin(GL_TRIANGLE_FAN);
		glVertex2f(x, y);
	}
	else
		glBegin(GL_LINE_LOOP);

	for (double angle = 0; angle <= 2 * M_PI; angle += M_PI / resolution)
		glVertex2f(x + sin(angle) * radius, y + cos(angle) * radius);
	glEnd();
}

static void beginTextDraw(QImage & fontImage)
{
	glGetDoublev(GL_CURRENT_COLOR, font_color);

	initFont(fontImage);

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, fontTexture);

	glBegin(GL_QUADS);
}

static void endTextDraw()
{
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
}
