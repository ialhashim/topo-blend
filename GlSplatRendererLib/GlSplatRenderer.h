#pragma once

#include "GlSplat/GlSplat.h"
#include "SurfaceMeshModel.h"

class GlSplatRenderer : public GlSplat::SplatRenderer
{
public:
    GlSplatRenderer(const QVector<Eigen::Vector3f> &pts, const QVector<Eigen::Vector3f> &ns,
                    double radius = 0.01, const Eigen::Vector4d & color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));

public:
    QVector<Eigen::Vector3f> points;
    QVector<Eigen::Vector3f> normals;
    double mRadius;
    double mColor[4];

    void draw();

//private:
    void drawpoints();
};
