#include "GlSplatRenderer.h"

GlSplatRenderer::GlSplatRenderer(double radius, const Eigen::Vector4d & color)
{
	Q_INIT_RESOURCE(shaders);

    mRadius = qMax(0.02,radius);
    for(int i = 0; i < 4; i++) mColor[i] = color[i];

	VertexVBOID = 0;
	VertexCount = 0;
}

void GlSplatRenderer::draw()
{
	if(!VertexCount) return;

    beginVisibilityPass();
    drawpoints();
    beginAttributePass();
    drawpoints();
    finalize();
}

void GlSplatRenderer::drawpoints()
{
	glColor4dv(mColor);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
	glMultiTexCoord1f(GL_TEXTURE2, mRadius);
	glNormalPointer(GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, nx));
	glVertexPointer(3, GL_FLOAT, sizeof(GLVertex), (void*)offsetof(GLVertex, x));
	glDrawArrays(GL_POINTS, 0, VertexCount);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void GlSplatRenderer::update( const std::vector<GLVertex> & splats )
{
	// Clean up previous
	if(VertexCount) glDeleteBuffers(1, &VertexVBOID);

	glGenBuffers(1, &VertexVBOID);
	glBindBuffer(GL_ARRAY_BUFFER, VertexVBOID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertex) * splats.size(), &splats[0].x, GL_STATIC_DRAW);

	VertexCount = splats.size();
}
