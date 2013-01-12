#include "qgl.h"
#include "Octree.h"
#include <stack>

Octree::Octree( int triPerNode, Surface_mesh * useMesh )
{
	this->parent = NULL;
	this->mesh = useMesh;
	this->trianglePerNode = triPerNode;

	// Using all tris
	std::vector<Surface_mesh::Face> allTris;
	Surface_mesh::Face_iterator fit, fend = useMesh->faces_end();
	for(fit = useMesh->faces_begin(); fit != fend; ++fit)
		allTris.push_back(fit);

	this->initBuild(allTris, trianglePerNode);
}

Octree::Octree( int triPerNode, const BoundingBox& bb, const std::vector<Surface_mesh::Face>& tris, Surface_mesh * useMesh )
{
	this->parent = NULL;
	this->mesh = useMesh;
	this->boundingBox = bb;
	this->trianglePerNode = triPerNode;
	this->triangleData = tris;
}

void Octree::initBuild( std::vector<Surface_mesh::Face>& tris, int triPerNode )
{
	// add triangles involved to "triangleData"
	this->triangleData = tris;
	this->trianglePerNode = triPerNode;

	// Create a big box
	BoundingBox bb;

	// Collect faces geometry
	points = mesh->vertex_property<Vec3d>("v:point");
	std::vector< std::vector<Vec3d> > triangles ( mesh->n_faces() );
	Surface_mesh::Face_iterator fit, fend = mesh->faces_end();
	for(fit = mesh->faces_begin(); fit != fend; ++fit){
		std::vector<Vec3d> pnts; 
		Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(fit),vend=vit;
		do{ pnts.push_back(points[vit]); } while(++vit != vend);
		triangles[Surface_mesh::Face(fit).idx()] = pnts;
	}

	bb.computeFromTris(triangles);

	// Transform and scale to node's coordinates
	double largeSize = qMax(bb.xExtent, qMax(bb.yExtent, bb.zExtent));

	largeSize *= 1.1;

	// Define our bounding box
	this->boundingBox = BoundingBox(bb.center, largeSize, largeSize, largeSize);

	// Build the tree
	this->build();

	// Connect children with parent
	std::stack<Octree*> childStack;
	childStack.push(this);
	while(!childStack.empty())
	{
		Octree * curr = childStack.top(); childStack.pop();

		for(int i = 0; i < (int) curr->children.size(); i++)
		{
			curr->children[i].parent = curr;

			childStack.push( &curr->children[i] );
		}
	}
}

void Octree::newNode( int depth, double x, double y, double z )
{
	double newExtent = boundingBox.xExtent / 2.0;

	Vec3d center;

	center.x() = boundingBox.center.x() + (newExtent * x);
	center.y() = boundingBox.center.y() + (newExtent * y);
	center.z() = boundingBox.center.z() + (newExtent * z);

	BoundingBox bb(center, newExtent, newExtent, newExtent);

	// Add child
	children.push_back(Octree());
	Octree * child = &children.back();

	child->mesh = mesh;
	child->points = points;
	child->boundingBox = bb;
	child->trianglePerNode = this->trianglePerNode;

	// Collect triangles inside child's bounding box
	for(std::vector<Surface_mesh::Face>::iterator it = this->triangleData.begin(); it != this->triangleData.end(); it++)
	{
		Surface_mesh::Face face = *it;

		std::vector<Vec3d> v = triPoints(face);

		if( bb.containsTriangle(v[0], v[1], v[2]) )
		{
			child->triangleData.push_back(face);
		}
	}

	child->build(depth + 1); // build it
}

std::vector<Surface_mesh::Face> Octree::getIntersectingTris(const Vec3d& v0, const Vec3d& v1, const Vec3d& v2, bool showIt)
{
	if(this->triangleData.size() == 0 || this->children.size() == 0)
		return this->triangleData;

	std::vector<Surface_mesh::Face> res;

	for(int i = 0; i < (int) this->children.size(); i++)
	{
		if(children[i].boundingBox.containsTriangle(v0, v1, v2))
		{
			const std::vector<Surface_mesh::Face> tris = children[i].getIntersectingTris(v0, v1, v2, showIt);

			for(int j = 0; j < (int) tris.size(); j++)
				res.push_back(tris[j]);
		}
	}

	return res;
}

void Octree::build( int depth )
{
	if ((int)triangleData.size() > this->trianglePerNode)
	{
		if(depth < 8)
		{
			// Subdivide to 8 nodes
			newNode(depth, -1, -1, -1);
			newNode(depth, 1, -1, -1);
			newNode(depth, -1, 1, -1);
			newNode(depth, 1, 1, -1);
			newNode(depth, -1, -1, 1);
			newNode(depth, 1, -1, 1);
			newNode(depth, -1, 1, 1);
			newNode(depth, 1, 1, 1);
		}
	}
}
void Octree::DrawBox(const Vec3d& center, float width, float length, float height, float r, float g, float b, float lineWidth)
{
	Vec3d  c1, c2, c3, c4;
	Vec3d  bc1, bc2, bc3, bc4;

	c1 = Vec3d (width, length, height) + center;
	c2 = Vec3d (-width, length, height) + center;
	c3 = Vec3d (-width, -length, height) + center;
	c4 = Vec3d (width, -length, height) + center;

	bc1 = Vec3d (width, length, -height) + center;
	bc2 = Vec3d (-width, length, -height) + center;
	bc3 = Vec3d (-width, -length, -height) + center;
	bc4 = Vec3d (width, -length, -height) + center;

	glDisable(GL_LIGHTING);

	glColor3f(r, g, b);
	glLineWidth(lineWidth);

	glBegin(GL_LINES);
	glVertex3dv(c1);glVertex3dv(bc1);
	glVertex3dv(c2);glVertex3dv(bc2);
	glVertex3dv(c3);glVertex3dv(bc3);
	glVertex3dv(c4);glVertex3dv(bc4);
	glVertex3dv(c1);glVertex3dv(c2);
	glVertex3dv(c3);glVertex3dv(c4);
	glVertex3dv(c1);glVertex3dv(c4);
	glVertex3dv(c2);glVertex3dv(c3);
	glVertex3dv(bc1);glVertex3dv(bc2);
	glVertex3dv(bc3);glVertex3dv(bc4);
	glVertex3dv(bc1);glVertex3dv(bc4);
	glVertex3dv(bc2);glVertex3dv(bc3);
	glEnd();
	glEnable(GL_LIGHTING);
}

void Octree::draw( double r, double g, double b, double lineWidth )
{
	if(root() == this)
	{
		foreach(Octree * iset, selectedChildren){
			//iset->draw(1,0,0,lineWidth + 3);
			BoundingBox bb = iset->boundingBox;
			DrawBox(bb.center, bb.xExtent, bb.yExtent, bb.zExtent,1,0,0, lineWidth + 2);
		}
	}

	DrawBox(boundingBox.center, boundingBox.xExtent, boundingBox.yExtent, boundingBox.zExtent,r,g,b, lineWidth);

	for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
		child->draw(r,g,b, lineWidth);
}

IndexSet Octree::intersectPoint( const Vec3d& point )
{
	IndexSet tris;

	if (boundingBox.contains(point)) 
		intersectRecursivePoint(point, tris);

	return tris;
}

void Octree::intersectRecursivePoint( const Vec3d& point, IndexSet& tris )
{
	if (intersectHit(tris))
		return;

	for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
	{
		if (child->boundingBox.contains(point))
			child->intersectRecursivePoint(point, tris);
	}
}

bool Octree::intersectHit( IndexSet& tris )
{
	if( this->children.size() > 0 )
		return false;

	for(std::vector<Surface_mesh::Face>::iterator it = triangleData.begin(); it != triangleData.end(); it++)
	{
		Surface_mesh::Face face = *it;
		tris.insert( face.idx() );
	}

	// Debug:
	//root()->selectedChildren.push_back(this);

	return true;
}

QSet<int> Octree::intersectRay( Ray ray, double rayThickness, bool isFullTest)
{
	QSet<int> tris;

	ray.thickness = rayThickness;

	//DEBUG:
	this->selectedChildren.clear();

	if ( this->boundingBox.intersects(ray) ) 
	{
		std::stack<Octree*> s;
		s.push( this );

		while( !s.empty() )
		{
			Octree * curTree = s.top();
			s.pop();

			if(curTree->children.size() == 0)
			{
				for(std::vector<Surface_mesh::Face>::iterator it = curTree->triangleData.begin(); it != curTree->triangleData.end(); it++)
				{
					Surface_mesh::Face face = *it;
					tris.insert( face.idx() );
				}

				// Debug:
				root()->selectedChildren.push_back(curTree);
			}

			// Do following if child size > 0
			for (std::vector<Octree>::iterator child = curTree->children.begin();  child != curTree->children.end(); child++)
			{
				if ( child->boundingBox.intersects(ray) )
				{
					s.push( &(*child) );
				}
			}
		}

		if(isFullTest)
		{
			QSet<int> exactSet;
			foreach(int i, tris)
			{
				HitResult hitRes;
				intersectionTest(Surface_mesh::Face(i), ray, hitRes, false);
				if(hitRes.hit) exactSet.insert(i);
			}
			return exactSet;
		}
	}

	return tris;
}

void Octree::intersectRecursiveRay( const Ray& ray, IndexSet& tris )
{
	if(children.size() == 0)
	{
		for(std::vector<Surface_mesh::Face>::iterator it = triangleData.begin(); it != triangleData.end(); it++)
		{
			Surface_mesh::Face face = *it;
			tris.insert( face.idx() );
		}

		// Debug:
		//root()->selectedChildren.push_back(this);
	}

	// Do following if child size > 0
	for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
	{
		if ( child->boundingBox.intersects(ray) )
		{
			child->intersectRecursiveRay(ray, tris);
		}
	}
}

IndexSet Octree::intersectSphere( const Vec3d& sphere_center, double radius )
{
	IndexSet tris;

	if (boundingBox.intersectsSphere(sphere_center, radius)) 
		intersectRecursiveSphere(sphere_center, radius, tris);

	return tris;
}

void Octree::intersectRecursiveSphere( const Vec3d& sphere_center, double radius, IndexSet& tris )
{
	if (intersectHit(tris))
		return;

	for (std::vector<Octree>::iterator child = children.begin();  child != children.end(); child++)
	{
		if (child->boundingBox.intersectsSphere(sphere_center, radius))
			child->intersectRecursiveSphere(sphere_center, radius, tris);
	}
}


bool Octree::testIntersectHit( const Ray& ray, HitResult & hitRes )
{
	if(this->children.size() > 0)
		return false;

	// Do actual intersection test
	for(std::vector<Surface_mesh::Face>::iterator face = triangleData.begin(); face != triangleData.end(); face++)
	{
		Surface_mesh::Face f = *face;
		intersectionTest(f, ray, hitRes, true);
		if(hitRes.hit)
			return true;
	}

	return false;
}

Octree * Octree::root()
{
	if(parent == NULL)
		return this;
	else
		return parent->root();
}

std::vector<Vec3d> Octree::triPoints(Surface_mesh::Face f)
{
	std::vector<Vec3d> pnts; 
	Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
	do{ pnts.push_back(points[vit]); } while(++vit != vend);
	return pnts;
}

void Octree::intersectionTest( Surface_mesh::Face f, const Ray & ray, HitResult & res, bool allowBack )
{
	std::vector<Vec3d> vert = triPoints(f);
	res.hit = false;
	res.distance = DBL_MAX;

	Vec3d v0 = vert[0];
	Vec3d v1 = vert[1];
	Vec3d v2 = vert[2];

    allowBack = allowBack;

    double EPSILON = 1e-8;
	Vec3d edge1 = v1 - v0;
	Vec3d edge2 = v2 - v0;
	Vec3d pvec = cross(ray.direction, edge2);
	float det = dot(edge1, pvec);
	if (det > -EPSILON && det < EPSILON) return;
	float invDet = 1 / det;
	Vec3d tvec = ray.origin - v0;
	double u = dot(tvec, pvec) * invDet;
	if (u < 0 || u > 1) return;
	Vec3d qvec = cross(tvec, edge1);
	double v = dot(ray.direction, qvec) * invDet;
	if (v < 0 || u + v > 1) return;


	// Got intersection
	res.distance = dot(edge2, qvec) * invDet;
	res.hit = true;
}

void Octree::intersectionTestOld( Surface_mesh::Face f, const Ray & ray, HitResult & res, bool allowBack )
{
	res.hit = false;
	res.distance = DBL_MAX;

	double EPS = 1e-10;

	std::vector<Vec3d> v = triPoints(f);
	
	Vec3d vertex1 = v[0];
	Vec3d vertex2 = v[1];
	Vec3d vertex3 = v[2];

	// Compute vectors along two edges of the triangle.
	Vec3d edge1 = vertex2 - vertex1;
	Vec3d edge2 = vertex3 - vertex1;

	// Compute the determinant.
	Vec3d directionCrossEdge2 = cross(ray.direction, edge2);

	double determinant = dot(edge1, directionCrossEdge2);

	// If the ray is parallel to the triangle plane, there is no collision.
	if (abs(determinant) < EPS)
		return;

	double inverseDeterminant = 1.0 / determinant;

	// Calculate the U parameter of the intersection point.
	Vec3d distanceVec3dtor = ray.origin - vertex1;
	double triangleU = dot(distanceVec3dtor, directionCrossEdge2);
	triangleU *= inverseDeterminant;

	// Make sure it is inside the triangle.
	if (triangleU < 0 - EPS || triangleU > 1 + EPS)
		return;

	// Calculate the V parameter of the intersection point.
	Vec3d distanceCrossEdge1 = cross(distanceVec3dtor, edge1);
	double triangleV = dot(ray.direction, distanceCrossEdge1);
	triangleV *= inverseDeterminant;

	// Make sure it is inside the triangle.
	if (triangleV < 0 - EPS || triangleU + triangleV > 1 + EPS)
		return;

	// Compute the distance along the ray to the triangle.
	double rayDistance = dot(edge2, distanceCrossEdge1);
	rayDistance *= inverseDeterminant;

	if(!allowBack){
		// Is the triangle behind the ray origin?
		if (rayDistance < 0)
			return;
	}

	res.hit = true;
	res.distance = rayDistance;

	res.u = triangleU;
	res.v = triangleV;

	res.index = f.idx();
}

#define VEC_FROM_POINTS(a,b,c) \
	(a)[0] = (b)[0] - (c)[0];	\
	(a)[1] = (b)[1] - (c)[1];	\
	(a)[2] = (b)[2] - (c)[2];

void Octree::my_intersectionTest( Surface_mesh::Face ff, const Ray & ray, HitResult & res, bool allowBack )
{
	std::vector<Vec3d> vert = triPoints(ff);
	res.hit = false;
	res.distance = DBL_MAX;

    allowBack = allowBack;

	Vec3d e1,e2,h,s,q;
	float a,f,u,v,t;
	Vec3d v0=vert[0];
	Vec3d v1=vert[1];
	Vec3d v2=vert[2];

	Vec3d origin=ray.origin;
	Vec3d direction=ray.direction;

	VEC_FROM_POINTS(e1,v1,v0);
	VEC_FROM_POINTS(e2,v2,v0);


	//crossProduct(h,direction,e2);
	h=cross(direction,e2);
	a = dot(e1,h);

	if (a > -0.00001 && a < 0.00001)
		return;

	f = 1/a;
	VEC_FROM_POINTS(s,origin,v0);
	u = f * (dot(s,h));

	if (u < 0.0 || u > 1.0)
		return;

	q=cross(s,e1);
	v = f * dot(direction,q);

	if (v < 0.0 || u + v > 1.0)
		return;

	// at this stage we can compute t to find out where
	// the intersection point is on the line
	t = f *dot(e2,q);

	if (t > 1e-6) // ray intersection
	{
		//intesect_point[0]=origin[0]+t*direction[0];
		//intesect_point[1]=origin[1]+t*direction[1];
		//intesect_point[2]=origin[2]+t*direction[2];

		//		intesect_point[0]=origin[0]+(1-u-v)*v0[0]+u*v1[0]+v*v2[0];
		//		intesect_point[1]=origin[1]+(1-u-v)*v0[1]+u*v1[1]+v*v2[1];
		//		intesect_point[2]=origin[2]+(1-u-v)*v0[2]+u*v1[2]+v*v2[2];

		res.distance=t*dot(direction,direction);
		res.hit=true;
		return;
	}
	else // this means that there is a line intersection
		// but not a ray intersection
		return;
}

//Surface_mesh::Face* Octree::findClosestTri( const Ray & ray, IndexSet & tris, Mesh * mesh, HitResult & hitRes )
//{
//	double minDist = DBL_MAX;
//	Surface_mesh::Face *closestFace = NULL, *curr_FaceType = NULL;

//	double u = 0.0, v = 0.0;
//	double actualMinDist = 0;

//	// Find the triangles in this tree
//	intersectRayBoth(ray, tris);

//	for(IndexSetIter it = tris.begin(); it != tris.end(); it++)
//	{
//		curr_FaceType = mesh->f(*it);

//		curr_FaceType->intersectionTest(ray, hitRes, true);

//		if(hitRes.hit && (abs(hitRes.distance) < minDist))
//		{
//			closestFace = curr_FaceType;

//			minDist = abs(hitRes.distance);
//			actualMinDist = hitRes.distance;

//			u = hitRes.u;
//			v = hitRes.v;
//		}
//	}

//	if(closestFace)
//	{
//		// set 'hitRes' to that closest hit result
//		hitRes.distance = actualMinDist;
//		hitRes.index = closestFace->index;
//		hitRes.u = u;	hitRes.v = v;
//		hitRes.hit = true;
//	}
//	else
//	{
//		hitRes.hit = false;
//		hitRes.distance = DBL_MAX;
//		hitRes.index = -1;
//	}

//	return closestFace;
//}
