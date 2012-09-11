#pragma once
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"
#include "RichParameterSet.h"

enum FaceClass {CURVE, SHEET};
typedef QSet<Face> Region;
typedef QVector<Region> RegionVector;

class segment : public SurfaceMeshFilterPlugin{
    Q_OBJECT
    Q_INTERFACES(FilterPlugin)

public:
    QString name() { return "Segment"; }
    QString description() { return "Segment into curves and sheets."; }

    void initParameters(RichParameterSet* pars);
    void applyFilter(RichParameterSet* pars);
	
	void initMesh();
	void GrowRegions();

	Vector3VertexProperty points;
	ScalarFaceProperty farea;
	ScalarEdgeProperty elen;
	ScalarVertexProperty vclass;

	// Classification & region growing
	Surface_mesh::Face_property<int> fclass;
	BoolFaceProperty fvisited;
	void resetClassfication();
	void resetVisitFlag();
	void resetVertexClass();

	RegionVector curve_regions;
	RegionVector sheet_regions;

	double minAngle(Face f);
	Region growRegion(Face f);
	
	void performCurveSheetSegmentation(double theta, int k, bool isExtract = true, bool isVisualize = false);
	void classifyVertsFromFaces();
	void collectRing(Vertex v, QSet<Vertex> & set, int level = 0);
	void setMeshFromRegion(Region & r, SurfaceMeshModel *);

	double median(QVector<double> vec);
};
