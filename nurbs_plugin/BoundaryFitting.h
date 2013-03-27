#pragma once

#include "SurfaceMeshModel.h"

#include "NanoKdTree.h"
#include "../CustomDrawObjects.h"

typedef QPair<Halfedge,double> HalfedgeTime;
typedef QMap<double, HalfedgeTime > DistanceHalfedge;

struct BoundaryFitting{

	BoundaryFitting::BoundaryFitting( SurfaceMeshModel * mesh = NULL, int numSegments = 20, int numSegmentsV = -1);
	void doFit();

	std::vector<SurfaceMesh::Vertex> collectRings( SurfaceMesh::Vertex v, size_t min_nb, bool isBoundary = false );
	
	QVector<SurfaceMesh::Vertex> boundaryVerts();

	void normalize( SurfaceMesh::ScalarVertexProperty & vprop );

	void gradientFaces( ScalarVertexProperty & functionVal, bool isSmooth = true, bool isNormalizeNegateGradient = true );
	QVector<SurfaceMesh::Vertex> neighbours( int start, int range, QVector<SurfaceMesh::Vertex> all );

	SurfaceMesh::Halfedge get_halfedge(SurfaceMesh::Vertex start, SurfaceMesh::Vertex end);
	SurfaceMesh::Halfedge getBestEdge(Vec3d prevPoint, Vec3d direction, Face f, double & bestTime);
	SurfaceMesh::Face getBestFace( Vec3d & point, Vertex guess );

	std::vector<Vec3d> geodesicPath(Vec3d fromPoint, Vec3d toPoint);
	std::vector< std::vector<Vec3d> > geodesicPaths( std::vector<Vec3d> fromPoints, std::vector<Vec3d> toPoints, int segments = -1 );

	std::vector<Vec3d> trianglePoints( Face f );
	std::vector<Vertex> triangleVertices( Face f );

	SurfaceMesh::SurfaceMeshModel * part;
	int segments, segmentsV;

	SurfaceMesh::ScalarVertexProperty vals;
	SurfaceMesh::ScalarVertexProperty boundaryCurv;
	SurfaceMesh::ScalarVertexProperty dists;
	SurfaceMesh::Vector3VertexProperty vgradient;

	SurfaceMesh::Vector3FaceProperty fgradient;
	SurfaceMesh::Vector3VertexProperty vdirection;
	SurfaceMesh::Vector3VertexProperty normals;
	SurfaceMesh::Vector3VertexProperty points;

	NanoKdTree tree;

	// output
	std::vector< std::vector<Vec3d> > lines;

	std::vector<Vec3d> debugPoints, debugPoints2;
	std::vector<Vec3d> corners;
	std::vector< std::vector<Vec3d> > main_edges;
	LineSegments ls;
};
