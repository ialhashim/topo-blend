#pragma once

#include "SurfaceMeshModel.h"

#include "NanoKdTree.h"

struct BoundaryFitting2{

	BoundaryFitting2(SurfaceMesh::SurfaceMeshModel * mesh = NULL);

	void doFit();

	std::vector<SurfaceMesh::Vertex> collectRings( SurfaceMesh::Vertex v, size_t min_nb, bool isBoundary = false );
	
	QVector<SurfaceMesh::Vertex> boundaryVerts();

	void normalize( SurfaceMesh::ScalarVertexProperty & vprop );

	void gradientFaces(bool isNormalizeNegateGradient = true);

	QVector<SurfaceMesh::Vertex> neighbours( int start, int range, QVector<SurfaceMesh::Vertex> all );
	SurfaceMesh::SurfaceMeshModel * part;

	SurfaceMesh::Halfedge get_halfedge(SurfaceMesh::Vertex start, SurfaceMesh::Vertex end);
	std::vector<Vec3d> geodesicPath(Vec3d fromPoint, Vec3d toPoint);
	SurfaceMesh::ScalarVertexProperty vals;
	SurfaceMesh::ScalarVertexProperty boundaryCurv;
	SurfaceMesh::ScalarVertexProperty dists;

	SurfaceMesh::Vector3FaceProperty fgradient;
	SurfaceMesh::Vector3VertexProperty vdirection;
	SurfaceMesh::Vector3VertexProperty normals;
	SurfaceMesh::Vector3VertexProperty points;

	NanoKdTree tree;

	std::vector<Vec3d> debugPoints;
};
