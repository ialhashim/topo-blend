#pragma once
#include "qglviewer.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

enum DrawElementType{VERT_IDX, FACE_IDX, EDGE_IDX, HDGE_IDX};

class surfacemesh_mode_info : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/cursor-question.png"); }
    QImage fontImage;

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();
    void drawWithNames();
	void postSelection(const QPoint& p);
	void endSelection(const QPoint& p);

	void drawIndex(DrawElementType, QColor, double vt = -0.4);

	void beginDrawIndex();
	void drawIndexVertex(Vertex,bool shadow=false);
	void drawIndexFace(Face,bool shadow=false);
	void drawIndexEdge(int,QVector3D,bool shadow=false);
	void endDrawIndex();

	void drawSelectedItem();
	void drawItemInfo();

	qglviewer::Vec cameraProjection(QVector3D);

	Vector3VertexProperty points;
	Vector3FaceProperty faceCenters;
	Vector3FaceProperty faceNormals;
	ScalarFaceProperty faceAreas;
	ScalarEdgeProperty elengs;

	DrawElementType selectedType;
	int selectedIdx;

	QVector<bool> visualize;

public:
	virtual bool keyReleaseEvent(QKeyEvent* event);
	virtual bool keyPressEvent (QKeyEvent* event);
	void update();
};

// Utility
#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )
#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)
