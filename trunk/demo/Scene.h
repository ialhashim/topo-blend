#pragma once
#include "DemoGlobal.h"
#include "GraphItem.h"

class Scene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit Scene(QObject *parent = 0);

	GraphItem * inputGraphs[2];
	qglviewer::Camera * camera;

    bool isInputReady() { return inputGraphs[0] && inputGraphs[1]; }

    QRect graphRect(bool isRight);

	QGraphicsProxyWidget * addButton( int x, int y, QString text, QImage icon = QImage() );

protected:
    void drawBackground ( QPainter * painter, const QRectF & rect );
    void drawForeground ( QPainter * painter, const QRectF & rect );
    void draw3D();

    void testScene(); // delete this later

protected:
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event );
    void mousePressEvent( QGraphicsSceneMouseEvent * event );
    void mouseMoveEvent( QGraphicsSceneMouseEvent * event );
    void wheelEvent( QGraphicsSceneWheelEvent *event );
	void mouseDoubleClickEvent( QGraphicsSceneMouseEvent * mouseEvent );
	void keyReleaseEvent(QKeyEvent * keyEvent);

private:
    void setupCamera();
    void setupLights();

signals:
	void mousePressDownEvent(QGraphicsSceneMouseEvent * event);
	void mousePressUpEvent(QGraphicsSceneMouseEvent * event);
	void keyUpEvent(QKeyEvent*);
    void wheelEvents(QGraphicsSceneWheelEvent*);
	void doubleClick();
	void rightClick();
	void message(QString);

public slots:
    
};

// Utility:
// Trackball code
static inline float projectOnBall(float x, float y)
{
	const float size       = 1.0f;
	const float size2      = size*size;
	const float size_limit = size2*0.5;
	const float d = x*x + y*y;
	return d < size_limit ? sqrt(size2 - d) : size_limit/sqrt(d);
}
static inline qglviewer::Quaternion deformedBallQuaternion(int prevX, int prevY, int x, int y, float cx, float cy,
	int viewportWidth, int viewportHeight, float rotationSensitivity = 1.0){
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

Q_DECLARE_METATYPE(qglviewer::Quaternion)
Q_DECLARE_METATYPE(qglviewer::Vec)
