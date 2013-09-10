#pragma once

#include <qglviewer/camera.h>
Q_DECLARE_METATYPE(qglviewer::Camera*)

#include <QObject>
#include <QGLWidget>
#include <QGraphicsItem>

// Forward declare
class SynthesisManager;
namespace Structure { struct Graph; }

class BlendPathRenderer : public QGLWidget
{
    Q_OBJECT
public:
    explicit BlendPathRenderer(SynthesisManager * synthManager, int itemHeight, QWidget *parent = 0);
    
protected:
	void initializeGL(); 
	void paintGL();

private:
	SynthesisManager * s_manager;
	Structure::Graph * activeGraph;

signals:
    void itemReady(QGraphicsItem *);

public slots:
    void generateItem(Structure::Graph*, int pathID, int blendIDX);
};
