#include "GlSplatRenderer.h"

GlSplatRenderer::GlSplatRenderer(const QVector<Vec3f> &pts, const QVector<Vec3f> &ns, double radius, Vec4d color)
{
	Q_INIT_RESOURCE(shaders);

    points = pts;
    normals = ns;
    mRadius = radius;
    for(int i = 0; i < 4; i++) mColor[i] = color[i];
}

void GlSplatRenderer::draw()
{
    beginVisibilityPass();
    drawpoints();
    beginAttributePass();
    drawpoints();
    finalize();
}

void GlSplatRenderer::drawpoints()
{
    glBegin(GL_POINTS);
    for (int i = 0; i < (int)points.size(); ++i)
    {
        glMultiTexCoord1f(GL_TEXTURE2, mRadius);
        glNormal3f(normals[i].x(), normals[i].y(), normals[i].z());
        glColor4dv(mColor);
        glVertex3f(points[i].x(), points[i].y(), points[i].z());
    }
    glEnd();
}
