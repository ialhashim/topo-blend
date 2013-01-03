#pragma once

#include <cmath>
#include <set>
#include <stack>
#include <vector>
#include <list>

#include "surface_mesh/Surface_mesh.h"
#include "BoundingBox.h"

typedef std::set<int> IndexSet;
typedef IndexSet::iterator IndexSetIter;

class Octree
{
private:
	std::vector<Octree> children;
	std::vector<Surface_mesh::Face> triangleData;
	Surface_mesh * mesh;
	Surface_mesh::Vertex_property<Vec3d> points;

public:
	BoundingBox boundingBox;
	int trianglePerNode;

	Octree(){ trianglePerNode = -1; parent = NULL; mesh = NULL; }
    Octree( int triPerNode, const BoundingBox& bb, const std::vector<Surface_mesh::Face>& tris, Surface_mesh * useMesh );
	Octree( int triPerNode, Surface_mesh * useMesh );

	void init(int triPerNode);
	void initBuild(std::vector<Surface_mesh::Face>& tris, int triPerNode );

	void newNode( int depth, double x, double y, double z );
	void build(int depth = 0);

    std::vector<Surface_mesh::Face> getIntersectingTris(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2, bool showIt=false);

	bool intersectHit(IndexSet& tris);

	IndexSet intersectPoint(const Vec3d& point);
	void intersectRecursivePoint(const Vec3d& point, IndexSet& tris);

	IndexSet intersectRay(const Ray& ray, bool isFullTest = true);
	void intersectRecursiveRay(const Ray& ray, IndexSet& tris);

	IndexSet intersectSphere(const Vec3d& sphere_center, double radius);
	void intersectRecursiveSphere(const Vec3d& sphere_center, double radius, IndexSet& tris);

	IndexSet intersectRaySphere(const Ray& ray, const Vec3d& sphere_center, double radius);
	void intersectRayBoth(const Ray& ray, IndexSet & tris);

    /* Perform intersection tests  */
	bool testIntersectHit(const Ray& ray, HitResult & hitRes);
	void testIntersectRayBoth(const Ray& ray, HitResult & hitRes);

	void intersectionTest( Surface_mesh::Face f, const Ray & ray, HitResult & res, bool allowBack = true );

	Octree * parent;
	Octree * root();

	void draw(double r, double g, double b, double lineWidth = 1.0);
	void DrawBox(const Vec3d& center, float width, float length, float height, float r, float g, float b, float lineWidth);

	std::vector<Vec3d> triPoints(Surface_mesh::Face f);

	//	BaseTriangle* findClosestTri(const Ray & ray, IndexSet & tris, Mesh * mesh, HitResult & hitRes);

	// Debug
	std::vector<Octree *> selectedChildren;
};
