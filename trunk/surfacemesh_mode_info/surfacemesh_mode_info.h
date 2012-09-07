#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

enum DrawElementType{VERT_IDX, FACE_IDX, EDGE_IDX, HDGE_IDX};

class surfacemesh_mode_info : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

    QIcon icon(){ return QIcon(":/images/cursor-question.png"); }

    /// Functions part of the EditPlugin system
    void createEdit();
    void destroyEdit(){}

    void decorate();
    void drawWithNames();
	void postSelection(const QPoint& p);
	void endSelection(const QPoint& p);

	void drawIndex(DrawElementType, QColor, double vt = -0.4);
	void drawSelectedItem(DrawElementType, QColor);

	Vector3FaceProperty faceCenters;
	Vector3FaceProperty faceNormals;

	int selectedType;
	int selectedIdx;

	QVector<bool> visualize;

public:
	virtual bool keyReleaseEvent(QKeyEvent* event);
	virtual bool keyPressEvent (QKeyEvent* event);
};
