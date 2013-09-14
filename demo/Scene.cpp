#include "Scene.h"

Q_DECLARE_METATYPE(qglviewer::Quaternion)
Q_DECLARE_METATYPE(qglviewer::Vec)

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
        glColor3d(0.2,0.2,0.2); glVertex2d(width,height); glVertex2d(0,height);
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

// Trackball code
static float projectOnBall(float x, float y)
{
    const float size       = 1.0f;
    const float size2      = size*size;
    const float size_limit = size2*0.5;
    const float d = x*x + y*y;
    return d < size_limit ? sqrt(size2 - d) : size_limit/sqrt(d);
}
static qglviewer::Quaternion deformedBallQuaternion(int prevX, int prevY, int x, int y, float cx, float cy,
                                                    int viewportWidth, int viewportHeight, float rotationSensitivity = 1.0)
{
    // Points on the deformed ball
    float px = rotationSensitivity * (prevX  - cx) / viewportWidth;
    float py = rotationSensitivity * (cy - prevY)  / viewportHeight;
    float dx = rotationSensitivity * (x - cx)	   / viewportWidth;
    float dy = rotationSensitivity * (cy - y)	   / viewportHeight;

    const qglviewer::Vec p1(px, py, projectOnBall(px, py));
    const qglviewer::Vec p2(dx, dy, projectOnBall(dx, dy));
    const qglviewer::Vec axis = cross(p2,p1);
    const float angle = 2.0 * asin(sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
    return qglviewer::Quaternion(axis, angle);
}

void Scene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if(event->buttons() & Qt::LeftButton)
    {
        if(property("camera-rotate").toBool())
        {
            QPointF startPos = event->buttonDownScenePos(Qt::LeftButton);
            QPointF currentPos = event->scenePos();

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
				if(inputGraphs[i]) inputGraphs[i]->pick( pos.x(), pos.y() );
			}
		}

		update();
	}

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
