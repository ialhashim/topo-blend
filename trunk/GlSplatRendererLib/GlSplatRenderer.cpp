#include "GlSplatRenderer.h"

GlSplatRenderer::GlSplatRenderer(double radius, const Eigen::Vector4d & color)
{
	Q_INIT_RESOURCE(shaders);

    mRadius = qMax(0.03,radius);
    for(int i = 0; i < 4; i++) mColor[i] = color[i];
}

void GlSplatRenderer::draw(const std::vector<GLVertex> & splats)
{
    beginVisibilityPass();
    drawpoints( splats );
    beginAttributePass();
    drawpoints( splats );
    finalize();
}

void GlSplatRenderer::drawpoints(const std::vector<GLVertex> & splats)
{
	glColor4dv(mColor);

    glBegin(GL_POINTS);
    for (int i = 0; i < (int)splats.size(); ++i)
    {
        glMultiTexCoord1f(GL_TEXTURE2, mRadius);
        glNormal3f(splats[i].nx, splats[i].ny, splats[i].nz);
        glVertex3f(splats[i].x, splats[i].y, splats[i].z);
    }
    glEnd();
}
