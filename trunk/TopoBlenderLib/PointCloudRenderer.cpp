#include "GL/GLee.h"
#include "PointCloudRenderer.h"
#include <QFile>

// Objects wide
QGLShaderProgram * PointCloudRenderer::program = new QGLShaderProgram;
bool PointCloudRenderer::programReady = false;

PointCloudRenderer::PointCloudRenderer(const QVector<Vec3d> &positions, const QVector<Vec3d> &normals, float radius)
{
    // Generate VBO
    QVector<SplatData> splats;
    for(int i = 0; i < (int)positions.size(); i++)
    {
        SplatData s = {
            QVector3D(positions[i][0],positions[i][1],positions[i][2]),
            QVector3D(normals[i][0],normals[i][1],normals[i][2]),
            radius
        };

        splats.push_back(s);
    }

    glGenBuffers(1, &vboIds);

    // Transfer splats data to VBO
    glBindBuffer(GL_ARRAY_BUFFER, vboIds);
    glBufferData(GL_ARRAY_BUFFER, splats.size() * sizeof(SplatData), splats.constData(), GL_DYNAMIC_DRAW);

    numSplats = splats.size();
}

void PointCloudRenderer::initShaders()
{
	Q_INIT_RESOURCE(shaders);

    program->addShaderFromSourceFile(QGLShader::Vertex, ":/shaders/pc_vertex.glsl");
    program->addShaderFromSourceFile(QGLShader::Fragment, ":/shaders/pc_fragment.glsl");
    program->link();

    PointCloudRenderer::programReady = true;
}

void PointCloudRenderer::draw( qglviewer::Camera* camera )
{
    if(!programReady) PointCloudRenderer::initShaders();

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable(GL_POINT_SPRITE);

    // Assign the shader
    program->bind();

    double mv[16], pj[16];
    camera->getModelViewMatrix(mv);
    camera->getProjectionMatrix(pj);

    // Calculate model view transformation
    QMatrix4x4 mv_matrix(mv);
    mv_matrix=mv_matrix.transposed();
    QMatrix4x4 pj_matrix(pj);
    pj_matrix=pj_matrix.transposed();

    // Set modelview-projection matrix
    program->setUniformValue( "mvMatrix", mv_matrix );
    program->setUniformValue( "normalMatrix", mv_matrix.normalMatrix() );
    program->setUniformValue( "mvpMatrix", pj_matrix * mv_matrix);

    // Using texture unit 0 which contains cube.png
    program->setUniformValue("texture", 0);

    // get the camera parameters to compute the splat size
    float n=camera->zNear();
    float f=camera->zFar();
    QVector3D eye(camera->position().x,camera->position().y,camera->position().z);
    float fov=camera->fieldOfView();
    int h=camera->screenHeight();

    program->setUniformValue( "near", n );
    program->setUniformValue( "far", f );
    program->setUniformValue( "eyePos", eye );
    program->setUniformValue( "foView", fov );
    program->setUniformValue( "screenH", h );

    // Tell OpenGL which VBOs to use
    glBindBuffer(GL_ARRAY_BUFFER, vboIds);

    // Offset for position
    int offset = 0;

    // Tell OpenGL programmable pipeline how to locate splat position data
    int posLocation = program->attributeLocation("vPosition");
    program->enableAttributeArray(posLocation);
    glVertexAttribPointer(posLocation, 3, GL_FLOAT, GL_FALSE, sizeof(SplatData), (const void *)offset);

    // Offset for normal
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate splat normal data
    int normalLocation = program->attributeLocation("vNormal");
    program->enableAttributeArray(normalLocation);
    glVertexAttribPointer(normalLocation, 3, GL_FLOAT, GL_FALSE, sizeof(SplatData), (const void *)offset);

    // Offset for size
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate splat scale data
    int scaleLocation = program->attributeLocation("vSize");
    program->enableAttributeArray(scaleLocation);
    glVertexAttribPointer(scaleLocation, 1, GL_FLOAT, GL_FALSE, sizeof(SplatData), (const void *)offset);

    glDrawArrays(GL_POINTS, 0, numSplats);

    // Unbind shader
    program->release();
}

void PointCloudRenderer::clearVBO()
{
    glDeleteBuffers(1, &vboIds);
}
