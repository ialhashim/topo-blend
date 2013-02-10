#pragma once

#include <QGLShaderProgram>
#include "surface_mesh/Surface_mesh.h"
#include "qglviewer/camera.h"

struct SplatData{
    QVector3D	position;
    QVector3D	normal;
    float		size;
};

class PointCloudRenderer
{
public:
    PointCloudRenderer(const QVector<Vec3d> & positions, const QVector<Vec3d> & normals, float radius);
    void draw( qglviewer::Camera* camera );

    static QGLShaderProgram* program;
    static bool programReady;
    static void initShaders();

    void clearVBO();

private:
    GLuint vboIds;
    int numSplats;
};
