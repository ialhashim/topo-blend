#pragma once

#include <qglviewer/camera.h>
Q_DECLARE_METATYPE(qglviewer::Camera*)

#include <QObject>
#include <QGLWidget>
#include <QGraphicsItem>

// Forward declare
class Blender; class BlendRenderItem;
namespace Structure { struct Graph; }

class BlendPathRenderer : public QGLWidget
{
    Q_OBJECT
public:
    explicit BlendPathRenderer(Blender * blender, int itemHeight, bool isViewer = false, QWidget *parent = 0);
    
	friend class BlendPathSubButton;
	friend class BlendPathSub;
	friend class Blender;

	QImage quickRender( Structure::Graph* graph, QColor color );

protected:
	void initializeGL(); 
	void paintGL();
	
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);

private:
	Blender * blender;
	Structure::Graph * activeGraph;
	BlendRenderItem * genItem( Structure::Graph* newGraph, int pathID, int blendIDX );

	bool isViewerMode;

signals:
    void itemReady(QGraphicsItem *);

public slots:
    void generateItem(Structure::Graph*, int pathID, int blendIDX);
};
