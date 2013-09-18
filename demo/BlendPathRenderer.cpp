#include <qglviewer/camera.h>
#include <QGraphicsPixmapItem>

#include "BlendPathRenderer.h"
#include "BlenderRenderItem.h"
#include "Blender.h"
#include "SynthesisManager.h"

BlendPathRenderer::BlendPathRenderer( Blender * blender, int itemHeight, QWidget *parent ) 
	: QGLWidget(parent), blender(blender), activeGraph(NULL)
{
	int w = itemHeight;
	int h = itemHeight;

	setMinimumSize(w,h);
	setMaximumSize(w,h);

	QGLFormat f;
	f.setAlpha(true);
	f.setSampleBuffers(true);
	QGLFormat::setDefaultFormat(f);
	this->setFormat(f);

#ifndef Q_OS_WIN
    show();
#endif
}

void BlendPathRenderer::generateItem( Structure::Graph* newGraph, int pathID, int blendIDX )
{
	this->makeCurrent();
	this->activeGraph = newGraph;

	this->updateGL();

	// Extract an image and create a QGraphicsPixmapItem
	BlenderRenderItem * pixmapItem = new BlenderRenderItem( QPixmap::fromImage(grabFrameBuffer(true)) );

	pixmapItem->pathID = pathID;
	pixmapItem->blendIDX = blendIDX;
	pixmapItem->setFlags( QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable ); // DEBUG

	emit( itemReady( pixmapItem ) );
}

void BlendPathRenderer::initializeGL()
{
	// Setup lights and material
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	// Specular lighting
	bool isSpecular = false;
	if( isSpecular )
	{
		float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
		glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);
		glMateriali(GL_FRONT, GL_SHININESS, 56);
	}
}

void BlendPathRenderer::paintGL()
{
	if(!activeGraph) return;

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	// Get camera
	SynthesisManager * s_manager = blender->s_manager.data();
	qglviewer::Camera * sceneCamera = s_manager->property("camera").value<qglviewer::Camera*>();
	if(!sceneCamera) return;

	// Setup viewport and camera
	if(sceneCamera->type() != qglviewer::Camera::ORTHOGRAPHIC) sceneCamera->setType(qglviewer::Camera::ORTHOGRAPHIC);
	int w = width(), h = height();
	glViewport( 0, 0, w, h );
	sceneCamera->setScreenWidthAndHeight(w,h);
	sceneCamera->loadProjectionMatrix();
	sceneCamera->loadModelViewMatrix();

	// Draw current graph
	s_manager->pointSize = 1.0;
	s_manager->color = QColor( 255, 180, 68 );
	s_manager->drawSynthesis( activeGraph );
}
