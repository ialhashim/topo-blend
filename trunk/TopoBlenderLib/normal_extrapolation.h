// Adapted from VCGLib
#pragma once

#include <queue>
#include "disjoint_set.h"
#include "SurfaceMeshHelper.h"
#include "NanoKdTree.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
using namespace Eigen;

class NormalExtrapolation
{
public:
	enum NormalOrientation {IsCorrect = 0, MustBeFlipped = 1, Guess = 2};

private:

	// Plane structure: identify a plain as a <center, normal> pair
	struct Plane
	{
		Plane() { center = Vector3(0); normal = Normal(0); };

		// Object functor: return the bounding-box enclosing a given plane
		inline void GetBBox(QBox3D	&bb) { bb = QBox3D(center, center); };

		Vector3	center;
		Normal	normal;
		int		index;
	};

	// Represent an edge in the Riemannian graph
	struct RiemannianEdge
	{
		RiemannianEdge(Plane *p = NULL, Scalar w = std::numeric_limits<Scalar>::max())  {plane = p; weight = w; }
		Plane	*plane;
		Scalar 	weight;
	};
	// Represent an edge in the MST tree
	struct MSTEdge
	{
		MSTEdge(Plane *p0 = NULL, Plane *p1 = NULL, Scalar w = std::numeric_limits<Scalar>::max()) {u = p0; v = p1; weight = w;};
		inline bool operator<(const MSTEdge &e) const {return weight<e.weight;}
		Plane			*u;
		Plane			*v;
		Scalar weight;
	};
	// Represent a node in the MST tree
	struct MSTNode
	{
		MSTNode(MSTNode* p=NULL)  {parent = p;}
		MSTNode					*parent;
		Vertex					vertex;
		std::vector< MSTNode* >	sons;
	};

	typedef std::vector< Plane > 		PlaneContainer;
	typedef PlaneContainer::iterator 	PlaneIterator;

public:

	static void ExtrapolateNormals(const std::vector<Vec3d> & pointcloud, QVector<Vec3d> & normals, 
									unsigned int k, const int root_index=-1, NormalOrientation orientation = Guess)
	{
		SurfaceMeshModel m("pointcloud.off", "pointcloud");
		foreach(Vec3d p, pointcloud) m.add_vertex(p);
		NormalExtrapolation::ExtrapolateNormals(&m, k, root_index, orientation);
		
		// Output
		Vector3VertexProperty out_normals = m.get_vertex_property<Vector3>(VNORMAL);
		foreach(Vertex v, m.vertices()) normals.push_back( out_normals[v] );
	}
	
	static void ExtrapolateNormals(SurfaceMeshModel * mesh, unsigned int k, const int root_index=-1, NormalOrientation orientation = Guess)
	{
		char message[256];

		k = qMin(k, mesh->n_vertices());

		Vector3VertexProperty points = mesh->get_vertex_property<Vector3>(VPOINT);
		Vector3VertexProperty normals = mesh->vertex_property<Vector3>(VNORMAL, Normal(0));

		QBox3D dataset_bb;
		foreach(Vertex v, mesh->vertices())	dataset_bb.unite(points[v]);
		Scalar max_distance = dataset_bb.size().length();

		// Step 1: identify the tangent planes used to locally approximate the surface
		int vertex_count = mesh->n_vertices();

		sprintf(message, "Locating tangent planes...");
		std::vector< Plane > tangent_planes(vertex_count);

		// KD-tree for finding local planes
		NanoKdTree kdtree_for_planes;
		foreach(Vertex v, mesh->vertices()) kdtree_for_planes.addPoint(points[v]);
		kdtree_for_planes.build();

		foreach(Vertex v, mesh->vertices())
		{
			KDResults matches;
			kdtree_for_planes.k_closest(points[v], k, matches);

			// for each vertex *iter, compute the centroid as avarege of the k-nearest vertices of *iter
			Plane *plane = &tangent_planes[ v.idx() ];
			for (unsigned int n = 0; n < k; n++)
				plane->center += points[Vertex(matches[n].first)];
			plane->center /= Scalar(k);

			// then, identity the normal associated to the centroid
			Matrix3d covariance_matrix = Matrix3d::Zero();
			for (unsigned int n = 0; n < k; n++)
			{
				Vector3 diff = points[ Vertex(matches[n].first) ] - plane->center;
				for (int i=0; i<3; i++)
					for (int j=0; j<3; j++)
						covariance_matrix(i,j) += diff[i] * diff[j];
			}

			SelfAdjointEigenSolver<Matrix3d> es(covariance_matrix);
			Matrix3d e_vectors = es.eigenvectors();
			Vector3d np = e_vectors.col(0).normalized();

			plane->normal = Vector3(np(0), np(1), np(2));
			plane->index = v.idx();

			normals[v] = plane->normal;
		}
		
		// Step 2: build the Riemannian graph, i.e. the graph where each point is connected to the k-nearest neighbors.
		dataset_bb.setToNull();
		PlaneIterator ePlane = tangent_planes.end();
		for (PlaneIterator iPlane = tangent_planes.begin(); iPlane != ePlane; iPlane++)
			dataset_bb.unite(iPlane->center);
		max_distance = dataset_bb.size().length();

		// KD-tree for finding local planes
		NanoKdTree kdtree_for_plane;
		foreach(Plane plane, tangent_planes) 
			kdtree_for_plane.addPoint(plane.center);
		kdtree_for_plane.build();

		std::vector< std::vector< RiemannianEdge > > riemannian_graph(vertex_count); //it's probably that we are wasting the last position...

		sprintf(message, "Building Riemannian graph...");
		for (PlaneIterator iPlane = tangent_planes.begin(); iPlane != ePlane; iPlane++)
		{
			KDResults nearest_planes;
			kdtree_for_plane.k_closest(iPlane->center, k, nearest_planes);

			for (unsigned int n = 0; n < k; n++)
			{
				int idx = nearest_planes[n].first;
				if (iPlane->index < idx)
					riemannian_graph[iPlane->index].push_back(RiemannianEdge(&tangent_planes[idx], 1.0 - abs(dot(iPlane->normal, tangent_planes[idx].normal))));
			}
		}

		// Step 3: compute the minimum spanning tree (MST) over the Riemannian graph (we use the Kruskal algorithm)
		std::vector< MSTEdge > E;
		std::vector< std::vector< RiemannianEdge > >::iterator iRiemannian = riemannian_graph.begin();
		std::vector< RiemannianEdge >::iterator iRiemannianEdge, eRiemannianEdge;

		for (int i=0; i<vertex_count; i++, iRiemannian++)
			for (iRiemannianEdge = iRiemannian->begin(), eRiemannianEdge = iRiemannian->end(); iRiemannianEdge != eRiemannianEdge; iRiemannianEdge++)
				E.push_back(MSTEdge(&tangent_planes[i], iRiemannianEdge->plane, iRiemannianEdge->weight));

		std::sort( E.begin(), E.end() );
		DisjointSet<Plane> planeset;

		for (std::vector< Plane >::iterator iPlane = tangent_planes.begin(); iPlane!=ePlane; iPlane++)
			planeset.MakeSet( &*iPlane );

		std::vector< MSTEdge >::iterator iMSTEdge = E.begin();
		std::vector< MSTEdge >::iterator eMSTEdge = E.end();
		std::vector< MSTEdge > unoriented_tree;
		Plane *u, *v;
		for (; iMSTEdge!=eMSTEdge; iMSTEdge++)
			if ((u = planeset.FindSet(iMSTEdge->u)) != (v = planeset.FindSet(iMSTEdge->v)))
				unoriented_tree.push_back( *iMSTEdge ), planeset.Union(u, v);
		E.clear();

		// compute for each plane the list of sorting edges
		std::vector< std::vector< int > > incident_edges(vertex_count);
		iMSTEdge = unoriented_tree.begin();
		eMSTEdge = unoriented_tree.end();

		//int mst_size = int(unoriented_tree.size());

		sprintf(message, "Building oriented graph...");
		for (; iMSTEdge != eMSTEdge; iMSTEdge++)
		{
			int u_index = int(iMSTEdge->u->index);
			int v_index = int(iMSTEdge->v->index);
			incident_edges[ u_index ].push_back( v_index ),
			incident_edges[ v_index ].push_back( u_index );
		}

		// Traverse the incident_edges vector and build the MST
		Surface_mesh::Vertex_iterator iCurrentVertex, iSonVertex;
		std::vector< MSTNode > MST(vertex_count);

		std::vector< Plane >::iterator iFirstPlane = tangent_planes.begin();
		std::vector< Plane >::iterator iCurrentPlane, iSonPlane;

		MSTNode *mst_root;
		int r_index = (root_index!=-1)? root_index : vertex_count * (double(rand()) / RAND_MAX);
		mst_root = &MST[ r_index ];
		mst_root->parent = mst_root; //the parent of the root is the root itself

		if (orientation == MustBeFlipped)
		{
			normals[iCurrentVertex] = normals[iCurrentVertex] * -1.0;
		}

		{ // just to limit the scope of the variable border
			std::queue< int > border;
			border.push(r_index);
			int maxSize		= 0;
			int	queueSize	= 0;
			sprintf(message, "Extracting the tree...");
			while ((queueSize=int(border.size())) > 0)
			{
				int current_node_index = border.front();  border.pop();

				MSTNode *current_node	= &MST[current_node_index];		// retrieve the pointer to the current MST node
				current_node->vertex	= Vertex(current_node_index);	// and associate it to the MST node

				std::vector< int >::iterator iSon = incident_edges[ current_node_index ].begin();
				std::vector< int >::iterator eSon = incident_edges[ current_node_index ].end();
				for (; iSon!=eSon; iSon++)
				{
					MSTNode *son = &MST[ *iSon ];
					if (son->parent == NULL) // the node hasn't been visited
					{
						son->parent = current_node;					// Update the MST nodes
						current_node->sons.push_back(son);
						border.push( *iSon );
					}
					maxSize = std::max<int>(maxSize, queueSize);
				}
			}
		}

		// and finally visit the MST tree in order to propagate normal flips
		{
			std::queue< MSTNode* > border;
			border.push(mst_root);
			sprintf(message, "Orienting normals...");
			int maxSize		= 0;
			int queueSize	= 0;
			while ((queueSize = int(border.size())) > 0)
			{
				MSTNode *current_node = border.front(); border.pop();
				for (int s = 0; s < int(current_node->sons.size()); s++)
				{
					if (dot(normals[current_node->vertex], normals[current_node->sons[s]->vertex]) < 0.0)
						normals[current_node->sons[s]->vertex] *= -1.0; // flip

					border.push( current_node->sons[s] );
					maxSize = std::max<int>(maxSize, queueSize);
				}
			}
		}

		// Assume no boundary, try to guess correct orientation looking from a corner
		if(orientation == Guess)
		{
			Vector3 direction = mesh->bbox().maximum() - mesh->bbox().center();
			Vector3 position = mesh->bbox().maximum() + direction * 10.0;

			// closet point on mesh
			KDResults match;
			kdtree_for_planes.k_closest(position, 1, match);
			Normal closestNormal = normals[Vertex(match[0].first)];

			if(dot(direction, closestNormal) < 0)
				foreach(Vertex v, mesh->vertices())
				normals[v] *= -1.0;
		}
	};


	static void SmoothNormalsUsingNeighbors(SurfaceMeshModel * mesh, const unsigned int k, bool usedistance)
	{
		Vector3VertexProperty points = mesh->get_vertex_property<Vector3>(VPOINT);
		Vector3VertexProperty normals = mesh->vertex_property<Vector3>(VNORMAL, Normal(0));

		QBox3D dataset_bb;
		foreach(Vertex v, mesh->vertices())	dataset_bb.unite(points[v]);
		//Scalar max_distance = dataset_bb.size().length();

		// Step 1: identify the tangent planes used to locally approximate the surface
		int vertex_count 	= mesh->n_vertices();
		char message[128];
		sprintf(message, "Locating neighbors...");

		// KD-tree for finding local planes
		NanoKdTree tree_for_neighbors;
		foreach(Vertex v, mesh->vertices()) tree_for_neighbors.addPoint(points[v]);
		tree_for_neighbors.build();
		
		std::vector< Normal > new_normals(vertex_count);

		foreach(Vertex v, mesh->vertices())
		{
			KDResults matches;
			tree_for_neighbors.k_closest(points[v], k, matches);

			// for each vertex *iter, compute the normal as avarege of the k-nearest vertices of *iter
			Normal normal_accum(0.0, 0.0, 0.0);

			Scalar dist_max = matches.back().second;
			
			for (unsigned int n = 0; n < k; n++){
				if(usedistance)
					normal_accum += (normals[Vertex(matches[n].first)] * matches[n].second / dist_max);
				else
					normal_accum += normals[Vertex(matches[n].first)];
			}

			new_normals[v.idx()] = normal_accum / k;
		}

		sprintf(message, "Assigning normals...");
		foreach(Vertex v, mesh->vertices())
			normals[v] = new_normals[v.idx()];
	};
};
