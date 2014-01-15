#include <qglviewer/camera.h>
#include <QGraphicsPixmapItem>

#include "BlendPathRenderer.h"
#include "BlendRenderItem.h"
#include "Blender.h"
#include "SynthesisManager.h"

#include "GraphExplorer.h"

BlendPathRenderer::BlendPathRenderer( Blender * blender, int itemHeight, bool isViewer, QWidget *parent ) 
	: QGLWidget(parent), blender(blender), isViewerMode(isViewer), activeGraph(NULL)
{
	int w = itemHeight;
	int h = itemHeight;

	setMinimumSize(w,h);
	setMaximumSize(w,h);

	// Placement off-screen
	int x = -w * 1.2;
	int y = 0;
	this->setGeometry(x,y,w,h);
	this->setWindowFlags( Qt::Popup );
	this->setMouseTracking( true );

	QGLFormat f;
	f.setAlpha(true);
	f.setSampleBuffers(true);
	QGLFormat::setDefaultFormat(f);
	this->setFormat(f);

#ifndef Q_OS_WIN
	show();
#endif
}

BlendRenderItem * BlendPathRenderer::genItem( Structure::Graph* newGraph, int pathID, int blendIDX )
{
	this->makeCurrent();
	this->activeGraph = newGraph;
	this->updateGL();

	// Extract an image and create a QGraphicsPixmapItem
	BlendRenderItem * pixmapItem = new BlendRenderItem( QPixmap::fromImage(grabFrameBuffer(true)) );
	pixmapItem->property["pathID"].setValue( pathID );
	pixmapItem->property["blendIDX"].setValue( blendIDX );
	pixmapItem->property["graph"].setValue( newGraph );

	// DEBUG:
	if( true )
	{
		if(newGraph->property.contains("scoreConnectivity"))
		{
			QPainter qPainter( &pixmapItem->pixmap );
			qPainter.drawText(0,10, QString::number(newGraph->property["scoreConnectivity"].toDouble()));
			qPainter.drawText(0,20, QString::number(newGraph->property["scoreSymLocal"].toDouble()));
			qPainter.drawText(0,30, QString::number(newGraph->property["scoreSymGlobal"].toDouble()));
			qPainter.drawText(0,50, QString::number(newGraph->property["score"].toDouble()));
		}
	}

	return pixmapItem;
}

void BlendPathRenderer::generateItem( Structure::Graph* newGraph, int pathID, int blendIDX )
{
	emit( itemReady( genItem(newGraph, pathID, blendIDX) ) );
}

QImage BlendPathRenderer::quickRender( Structure::Graph* graph, QColor color )
{
	this->makeCurrent();
	this->activeGraph = graph;
	this->blender->s_manager->color = color;
	this->updateGL();
	return grabFrameBuffer(true);
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

	if( !isViewerMode ) 
		glClearColor(0, 0, 0, 0);
	else
		glClearColor(0.1f, 0.1f, 0.1f, 1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	// Get camera
	SynthesisManager * s_manager = blender->s_manager.data();
	qglviewer::Camera * sceneCamera = s_manager->property["camera"].value<qglviewer::Camera*>();
	if(!sceneCamera) return;

	// Setup viewport and camera
	if(sceneCamera->type() != qglviewer::Camera::ORTHOGRAPHIC) sceneCamera->setType(qglviewer::Camera::ORTHOGRAPHIC);

	int w = width(), h = height();
	glViewport( 0, 0, w, h );
	sceneCamera->setScreenWidthAndHeight(w,h);
	sceneCamera->loadProjectionMatrix();
	sceneCamera->loadModelViewMatrix();

	// Render options
	if( !isViewerMode ) s_manager->pointSize = 1.0;
	else s_manager->pointSize = 2.5;
	s_manager->color = QColor( 255, 180, 68 );

	// Draw current graph
	s_manager->drawSynthesis( activeGraph );

	if( !isViewerMode ) 
		s_manager->bufferCleanup();
	else{
		// Setup 2D
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();	glLoadIdentity();
		glOrtho(0, w, h, 0, 0.0, -1.0);
		glMatrixMode(GL_MODELVIEW);	glPushMatrix();	glLoadIdentity();

		// Draw border
		glLineWidth(5);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); glDisable(GL_LIGHTING);
		glColorQt( QColor(255, 180, 68).darker() ); 
		glBegin(GL_QUADS);
		glVertex2d(0,0); glVertex2d(w,0); glVertex2d(w,h); glVertex2d(0,h);
		glEnd();
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// End 2D
		glMatrixMode(GL_PROJECTION); glPopMatrix();
		glMatrixMode(GL_MODELVIEW); glPopMatrix();
	}
}

void BlendPathRenderer::mouseMoveEvent(QMouseEvent *event)
{
	QRect r(0, 0, width(), height());

	if(event->buttons() == Qt::NoButton) 
	{
		if(!r.contains(event->pos())) { hide(); return; }
	}
	else
	{
		if(event->buttons() & Qt::LeftButton)
		{
			SynthesisManager * s_manager = blender->s_manager.data();
			qglviewer::Camera * sceneCamera = s_manager->property["camera"].value<qglviewer::Camera*>();

			QPointF startPos = property("buttonDownPos").toPointF();
			QPointF currentPos = event->posF();

			// Reset
			sceneCamera->frame()->setPosition( sceneCamera->property("startPos").value<qglviewer::Vec>() );
			sceneCamera->frame()->setOrientation( sceneCamera->property("startOrientation").value<qglviewer::Quaternion>() );

			// Rotate to new view
			qglviewer::Quaternion rot = deformedBallQuaternion(startPos.x(), startPos.y(),
				currentPos.x(), currentPos.y(),	r.center().x(), r.center().y(), r.width(), r.height());
			sceneCamera->frame()->rotateAroundPoint(rot, sceneCamera->revolveAroundPoint());

			update();
		}

		if(event->buttons() & Qt::RightButton)
		{
			update();
			grabFrameBuffer(true).save(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss") + ".png");
		}
	}

	blender->s->update();

	QGLWidget::mouseMoveEvent(event);
}

void BlendPathRenderer::mousePressEvent(QMouseEvent *event)
{
	SynthesisManager * s_manager = blender->s_manager.data();
	qglviewer::Camera * sceneCamera = s_manager->property["camera"].value<qglviewer::Camera*>();

	QVariant pos, orient;
	pos.setValue( sceneCamera->frame()->position() );
	orient.setValue( sceneCamera->frame()->orientation() );
	sceneCamera->setProperty("startPos", pos);
	sceneCamera->setProperty("startOrientation", orient);

	setProperty("buttonDownPos", event->posF());

	if(event->buttons() & Qt::RightButton)
	{
		GraphExplorer * ge = new GraphExplorer;
		ge->update( activeGraph );
		ge->show();
	}

	QGLWidget::mousePressEvent(event);
}
