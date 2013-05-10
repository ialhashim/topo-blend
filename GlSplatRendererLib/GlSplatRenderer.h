#pragma once

#include "GlSplat/GlSplat.h"
#include "SurfaceMeshModel.h"

class GlSplatRenderer : public GlSplat::SplatRenderer
{
public:
    GlSplatRenderer(const QVector<Vec3f> &pts, const QVector<Vec3f> &ns,
                    double radius = 0.01, Vec4d color = Vec4d(1.0, 1.0, 1.0, 1.0));

public:
    QVector<Vec3f> points;
    QVector<Vec3f> normals;
    double mRadius;
    double mColor[4];

    void draw();

//private:
    void drawpoints();
};
