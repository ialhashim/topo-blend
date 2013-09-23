#pragma once
#include <QGLWidget>
#include <QVector3D>
#include "GLVertex.h"

class ShapeRenderer : public QGLWidget
{
    Q_OBJECT
public:
    explicit ShapeRenderer(QString filename, QColor color, int resolution);
    static QPixmap render(QString filename);

protected:
    void initializeGL();
    void paintGL();
	void setupCamera();

    QColor color;
    std::vector< GLVertex > vertices;
    std::vector< GLuint > indices;
	QVector3D bmin,bmax;
};
