#include "Scene.h"

Scene::Scene(QObject *parent) : QGraphicsScene(parent)
{
	inputGraphs[0] = inputGraphs[1] = NULL;

	this->setSceneRect(0, 0, 1280, 720);
	this->setupCamera();

	this->setProperty("camera-rotate", true);
}

void Scene::setupCamera()
{
	double worldRadius = 10.0;

	this->camera = new qglviewer::Camera;
	this->camera->setSceneRadius( worldRadius );
	this->camera->showEntireScene();
	this->camera->setUpVector(qglviewer::Vec(0,0,1));
	this->camera->setPosition(qglviewer::Vec(2,-2,2));
	this->camera->lookAt(qglviewer::Vec());
}

void Scene::setupLights()
{
	GLfloat lightColor[] = {0.9f, 0.9f, 0.9f, 1.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	// Specular
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);
	glMateriali(GL_FRONT, GL_SHININESS, 56);
}

void Scene::drawBackground(QPainter *painter, const QRectF &rect)
{
	int width = rect.width();
	int height = rect.height();

	// Background
	bool isDrawFancyBackground = true;
	if( isDrawFancyBackground )
	{
		painter->beginNativePainting();

		// Setup 2D
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, width, height, 0, 0.0, -1.0);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		// Draw gradient quad
		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		glColor3d(0,0,0); glVertex2d(0,0); glVertex2d(width,0);
		glColor3d(0.15,0.15,0.15); glVertex2d(width,height); glVertex2d(0,height);
		glEnd();

		// End 2D
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		// Background has no depth
		glClear(GL_DEPTH_BUFFER_BIT);

		painter->endNativePainting();
	}

	// Typical OpenGL drawing
	{
		painter->beginNativePainting();

		this->setupLights();

		glEnable(GL_MULTISAMPLE);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_DEPTH_TEST);

		glViewport( 0, 0, GLint(width), GLint(height) );
		camera->setScreenWidthAndHeight(width,height);

		draw3D();

		// Scene has no depth
		glClear(GL_DEPTH_BUFFER_BIT);
		glDisable(GL_DEPTH_TEST);

		painter->endNativePainting();
	}
}

QRect Scene::graphRect(bool isRight)
{
	int itemWidth = property("itemWidth").toInt();
	int innerWidth = width() - (2 * itemWidth);
	int graphWidth = innerWidth * 0.5;
	int graphHeight = height() - (0.2 * height());

	int x = (width() * 0.5) - graphWidth;
	int y = (height() * 0.5) - (graphHeight * 0.5);

	QRect r(x,y,graphWidth,graphHeight);
	if(isRight) r.translate(r.width(),0);
	return r;
}

void Scene::drawForeground(QPainter *painter, const QRectF &rect)
{
	QGraphicsScene::drawForeground(painter,rect);

	for(int i = 0; i < 2; i++)
	{
		if(inputGraphs[i] != NULL) continue;

		painter->save();
		painter->setPen(Qt::white);
		QRect r = graphRect(i);
		painter->drawText(r, Qt::AlignCenter, "Loading..");
		//painter->drawRect(r);
		painter->restore();
	}
}

void Scene::draw3D()
{
	camera->loadProjectionMatrix();
	camera->loadModelViewMatrix();

	//testScene();

	// Draw input graphs
	for(int i = 0; i < 2; i++)
	{
		if(inputGraphs[i]) inputGraphs[i]->draw3D( );
	}
}

void Scene::testScene()
{
	// Default camera:
	/*glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-2, 2, -1.5, 1.5, 1, 40);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, 0, -3);
	glRotatef(50, 1, 0, 0);
	glRotatef(70, 0, 1, 0);*/

	// Draw a white grid "floor" for the tetrahedron to sit on.
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINES);
	for (GLfloat i = -2.5; i <= 2.5; i += 0.25) {
		glVertex3f(i, 2.5, 0); glVertex3f(i, -2.5, 0);
		glVertex3f(2.5, i, 0); glVertex3f(-2.5, i, 0);
	}
	glEnd();

	// Draw the tetrahedron.
	glBegin(GL_TRIANGLE_STRIP);
	glColor3f(1, 1, 1); glVertex3f(0, 0, 2);
	glColor3f(1, 0, 0); glVertex3f(-1, 1, 0);
	glColor3f(0, 1, 0); glVertex3f(1, 1, 0);
	glColor3f(0, 0, 1); glVertex3f(0, -1.4f, 0);
	glColor3f(1, 1, 1); glVertex3f(0, 0, 2);
	glColor3f(1, 0, 0); glVertex3f(-1, 1, 0);
	glEnd();
}

void Scene::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	emit( mousePressUpEvent(event) );
	QGraphicsScene::mousePressEvent(event);
}

void Scene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF startPos = event->buttonDownScenePos(Qt::LeftButton);
	QPointF currentPos = event->scenePos();

	QList<QGraphicsItem*> underMouse = items(event->scenePos());
	bool isGraphUnderMouse = underMouse.contains(inputGraphs[0]) || underMouse.contains(inputGraphs[1]);
	
	if( isGraphUnderMouse )
	{
		if(event->buttons() & Qt::LeftButton && event->modifiers() == Qt::ShiftModifier)
		{
			int i = 0;
			if(startPos.x() > width() * 0.5) i = 1;

			if(inputGraphs[i]) 
			{
				inputGraphs[i]->pick( currentPos.x(), currentPos.y(), 1 );
				update();
			}
		}
		else if(event->buttons() & Qt::LeftButton)
		{
			if(property("camera-rotate").toBool())
			{
				// Reset
				camera->frame()->setPosition( camera->property("startPos").value<qglviewer::Vec>() );
				camera->frame()->setOrientation( camera->property("startOrientation").value<qglviewer::Quaternion>() );

				// Rotate to new view
				QRect r = graphRect(0);
				if(startPos.x() > width() * 0.5) r = graphRect(1);

				qglviewer::Quaternion rot = deformedBallQuaternion(startPos.x(), startPos.y(),
					currentPos.x(), currentPos.y(),
					r.center().x(), r.center().y(), r.width(), r.height());
				camera->frame()->rotateAroundPoint(rot, camera->revolveAroundPoint());

				update();
			}
		}

	}

	QGraphicsScene::mouseMoveEvent(event);
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	// Save current camera state
	if(property("camera-rotate").toBool())
	{
		QVariant pos, orient;
		pos.setValue( camera->frame()->position() );
		orient.setValue( camera->frame()->orientation() );
		camera->setProperty("startPos", pos);
		camera->setProperty("startOrientation", orient);
	}

	// Picking on graphs
	if(property("graph-pick").toBool())
	{
		if(event->button() == Qt::RightButton)
			emit( rightClick() );

		if(event->button() == Qt::LeftButton)
		{
			for(int i = 0; i < 2; i++)
			{
				QPointF pos = event->scenePos();
				if(inputGraphs[i]) inputGraphs[i]->pick( pos.x(), pos.y(), 0 );
			}
		}

		update();
	}

	emit( mousePressDownEvent(event) );
	QGraphicsScene::mousePressEvent(event);
}

void Scene::wheelEvent(QGraphicsSceneWheelEvent * event)
{
	if(property("camera-zoom").toBool())
	{
		QGraphicsScene::wheelEvent(event);
		camera->frame()->translate(camera->frame()->inverseTransformOf(qglviewer::Vec(0.0, 0.0, 0.1*camera->flySpeed()*event->delta())));
		update();
	}
	else
		emit( wheelEvents(event) );

	QGraphicsScene::wheelEvent(event);
}

void Scene::mouseDoubleClickEvent( QGraphicsSceneMouseEvent * mouseEvent )
{
	if(property("graph-pick").toBool())
	{
		emit( doubleClick() );
		update();
	}

	QGraphicsScene::mouseDoubleClickEvent(mouseEvent);
}

void Scene::keyReleaseEvent( QKeyEvent * keyEvent )
{
	emit( keyUpEvent(keyEvent) );
	QGraphicsScene::keyReleaseEvent(keyEvent);
}

QGraphicsProxyWidget * Scene::addButton( int x, int y, QString text, QImage icon )
{
	QString buttonStyle = "QPushButton {background:rgba(90,90,90,20);border: 3px solid rgba(90,90,90,0); \
						  padding: 9px;color: white;font-size: 12px; min-width: 50px;} \
						  QPushButton:hover{ background:rgb(255,153,0); border-color:rgb(255,153,0); } \
						  QPushButton:pressed{ background:rgb(255,153,0); border-color:#FFB444; top:2px; }";

	QPushButton * button = new QPushButton(text);
	button->setStyleSheet(buttonStyle);
	if(icon.width()) button->setIcon(QPixmap::fromImage(icon));

	QGraphicsProxyWidget * item = new QGraphicsProxyWidget;
	item->setWidget(button);
	item->setPos(x,y);

	addItem(item);
	return item;
}
