#pragma once

#include "GlSplat/GlSplat.h"
#include "SurfaceMeshModel.h"
#include "GLVertex.h"

class GlSplatRenderer : public GlSplat::SplatRenderer
{
public:
    GlSplatRenderer(double radius = 0.01, const Eigen::Vector4d & color = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0));

public:
    double mRadius;
    double mColor[4];

    void draw(const std::vector<GLVertex> & splats);

private:
    void drawpoints(const std::vector<GLVertex> & splats);
};
