#pragma once

#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"
#include "StarlabDrawArea.h"

#include "NURBSCurve.h"
#include "NURBSRectangle.h"
#include "NurbsDraw.h"

#include "nurbstools.h"

#include "OBB_Volume.h"

#include "../CustomDrawObjects.h"

namespace Structure{
	struct Graph;
}

class nurbs_plugin : public SurfaceMeshModePlugin{
    Q_OBJECT
    Q_INTERFACES(ModePlugin)

public:
	nurbs_plugin() { widget = NULL; }
    QIcon icon(){ return QIcon(":/images/nurbs_icon.png"); }

    /// Functions part of the EditPlugin system
    void create();
    void destroy(){}

    void decorate();

    std::vector<NURBS::NURBSCurved> curves;
    std::vector<NURBS::NURBSRectangled> rects;
	Structure::Graph * graph;

    NURBSTools * widget;

    Vector3VertexProperty points;

	OBB_Volume mesh_obb;

	SurfaceMesh::SurfaceMeshModel * entireMesh;
	Vector3VertexProperty entirePoints;

	PolygonSoup ps;

	NURBS::NURBSCurved curveFit( SurfaceMeshModel * part );
	NURBS::NURBSRectangled surfaceFit( SurfaceMeshModel * part );

	bool keyPressEvent( QKeyEvent* event );

	// Selection
    void drawWithNames();
    bool postSelection( const QPoint& point );

	// Groups
	QMap< QString, QVector<int> > groupFaces;
	QMap< int, QString > faceGroup;
	void loadGroupsFromOBJ();
	
	SurfaceMeshModel * extractMesh( QString gid );
	Vector3d pole( Vector3d center, double radius, SurfaceMeshModel * part );
	std::vector<Vector3d> sampleEdgeTri( Vector3d a, Vector3d b, Vector3d c );
	double minAngle(Face f, SurfaceMeshModel * ofMesh);
	std::vector<Vertex> collectRings(SurfaceMeshModel * part, Vertex v, size_t min_nb);

	void selectGroup(QString gid);

public slots:
	void clearAll();
	void saveAll();
	void loadGraph();

	void buildSamples();

	void prepareSkeletonize();
	void stepSkeletonizeMesh();

	void convertToCurve();
	void convertToSheet();

	void updateDrawArea();

	void experiment();
};
