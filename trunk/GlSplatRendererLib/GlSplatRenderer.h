#pragma once

#include "GlSplat/GlSplat.h"
#include "SurfaceMeshModel.h"

class GlSplatRenderer : public GlSplat::SplatRenderer
{
public:
    GlSplatRenderer(const QVector<Vec3d> &pts, const QVector<Vec3d> &ns,
                    double radius = 0.01, Vec4d color = Vec4d(1.0, 1.0, 1.0, 1.0));

public:
    QVector<Vec3d> points;
    QVector<Vec3d> normals;
    double mRadius;
    double mColor[4];

    void draw();

//private:
    void drawpoints();
};
