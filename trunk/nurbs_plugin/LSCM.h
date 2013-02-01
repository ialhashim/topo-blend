#pragma once

#include "SurfaceMeshModel.h"
#include "SurfaceMeshTypes.h"

using namespace SurfaceMeshTypes;

#include "PCA3.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
using namespace Eigen;

struct LSCM
{
	LSCM( SurfaceMeshModel * useMesh )
	{
		if(!useMesh->is_triangle_mesh()){
			qDebug() << "WARNING: mesh has been triangulated!";
			useMesh->triangulate();
		}

		this->mesh = useMesh;
		this->points = mesh->vertex_property<Vector3>(VPOINT);
		this->is_locked = mesh->vertex_property<bool>("v:locked");
		this->tex_coord = mesh->add_vertex_property<Vec2d>("v:texture", Vec2d(0,0));

		// Prepare
		get_bounding_box();
		Halfedge h = largest_border() ;
		principal_axes(h, center, V1, V2) ;

		// Locate two furthest points on largest boundary
		get_border_extrema(h, center, V1, vx_min, vx_max) ;
		
		this->rearange_vertices();

		this->setup_U();

		this->setup_A_b();

		this->solve();
		
		// copy the solution to the texture coordinates
		int vc = 0;
		foreach(Vertex vert, mesh->vertices()){
			if(is_locked[vert]) continue;

			double u = x[2 * vc + 0];
			double v = x[2 * vc + 1];
			tex_coord[ vert ] = Vec2d( u, v );

			vc++;
		}
	}

	void rearange_vertices()
	{
		// Find number of pinned vertices
		num_locked = 0;
		foreach(Vertex v, mesh->vertices()) if(is_locked[v]) num_locked++;

		// Remap vertex index based on free or pinned
		int l = 0, f = 0, newIdx = 0;
		foreach(Vertex v, mesh->vertices()) 
		{
			if(is_locked[v]) 
				newIdx = l++;
			else
				newIdx = f++;

			v_idx[v.idx()] = newIdx;
		}
	}

	void setup_U()
	{
		U.resize(2 * num_locked);

		int c = 0;
		foreach(Vertex v, mesh->vertices()) 
		{
			if(!is_locked[v]) continue;
			U[c + 0] = tex_coord[v][0];
			U[c + 1] = tex_coord[v][1];

			c += 2;
		}
	}

	void setup_A_b() 
	{
		const uint vertexCount = mesh->n_vertices();
		const uint D = 2 * (vertexCount /*- 2*/);
		const uint N = 2 * mesh->n_faces();

		Mf = Eigen::SparseMatrix<double>(N, 2 * (vertexCount - num_locked));
		Mp = Eigen::SparseMatrix<double>(N, 2 * (num_locked));
		b = Eigen::VectorXd::Zero(N);
		x = Eigen::VectorXd::Zero(D);

		typedef Eigen::Triplet<Scalar> T;
		std::vector< T > L, Lf, Lp;

		foreach( Face f, mesh->faces() )
		{
			std::vector<Vertex> vface;
			Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
			do{ vface.push_back( Vertex(vit) ); } while(++vit != vend);

			int id0 = vface[0].idx() ;
			int id1 = vface[1].idx() ;
			int id2 = vface[2].idx() ;

			Vec3d p0 = points[vface[0]] ; 
			Vec3d p1 = points[vface[1]] ;
			Vec3d p2 = points[vface[2]] ;

			normalize_uv(p0) ;
			normalize_uv(p1) ;
			normalize_uv(p2) ;

			Vec2d z0,z1,z2 ;
			project_triangle(p0,p1,p2,z0,z1,z2) ;
			Vec2d z01 = z1 - z0 ;
			Vec2d z02 = z2 - z0 ;
			double a = z01.x() ;
			double b = z01.y() ;
			double c = z02.x() ;
			double d = z02.y() ;
			assert(b == 0.0) ;

			// Note  : 2*id + 0 --> u
			//         2*id + 1 --> v
			int u0_id = 2*v_idx[id0] + 0 ;
			int v0_id = 2*v_idx[id0] + 1 ;

			int u1_id = 2*v_idx[id1] + 0 ;
			int v1_id = 2*v_idx[id1] + 1 ;

			int u2_id = 2*v_idx[id2] + 0 ;
			int v2_id = 2*v_idx[id2] + 1 ;

			// Note : b = 0
			int row = f.idx();

			// Real part
			((isPinned(id0)) ? Lp : Lf ).push_back(T(2*row + 0, u0_id, -a+c));
			((isPinned(id0)) ? Lp : Lf ).push_back(T(2*row + 0, v0_id,  b-d));
			((isPinned(id1)) ? Lp : Lf ).push_back(T(2*row + 0, u1_id,   -c));
			((isPinned(id1)) ? Lp : Lf ).push_back(T(2*row + 0, v1_id,	  d));
			((isPinned(id2)) ? Lp : Lf ).push_back(T(2*row + 0, u2_id,	  a));

			// Imaginary part
			((isPinned(id0)) ? Lp : Lf ).push_back(T(2*row + 1, u0_id, -b+d));
			((isPinned(id0)) ? Lp : Lf ).push_back(T(2*row + 1, v0_id, -a+c));
			((isPinned(id1)) ? Lp : Lf ).push_back(T(2*row + 1, u1_id,   -d));
			((isPinned(id1)) ? Lp : Lf ).push_back(T(2*row + 1, v1_id,   -c));
			((isPinned(id2)) ? Lp : Lf ).push_back(T(2*row + 1, v2_id,    a));
		}

		Mf.setFromTriplets(Lf.begin(), Lf.end());
		Mp.setFromTriplets(Lp.begin(), Lp.end());

		A = Mf;
		b = -Mp * U;
	}

	void solve()
	{
		At = A.transpose();
		Eigen::CholmodSupernodalLLT< Eigen::SparseMatrix<double> > solver(At * A);
		x = solver.solve(At * b);
	}

	inline bool isPinned(int vidx)
	{
		return is_locked[Vertex(vidx)];
	}

	static void project_triangle(const Vec3d& p0, const Vec3d& p1, const Vec3d& p2, Vec2d& z0, Vec2d& z1, Vec2d& z2) 
	{
		Vec3d X = (p1 - p0).normalized();
		Vec3d Z = (cross(X, p2 - p0)).normalized();
		Vec3d Y = cross(Z, X);
		const Vec3d& O = p0;

		double x0 = 0 ;
		double y0 = 0 ;
		double x1 = (p1 - O).norm() ;
		double y1 = 0 ;
		double x2 = dot(p2 - O, X) ;
		double y2 = dot(p2 - O, Y) ;        

		z0 = Vec2d(x0,y0) ;
		z1 = Vec2d(x1,y1) ;
		z2 = Vec2d(x2,y2) ;        
	}

	void get_bounding_box()
	{
		// Compute center
		Vec3d sum(0.0);
		foreach(Vertex v, mesh->vertices())	sum += points[v];		
		center_ = sum / mesh->n_vertices();

		// Compute radius
		radius_ = 0;
		foreach(Vertex v, mesh->vertices()){
			double r = (points[v] - center_).norm() ;
			radius_ = qMax(radius_, r) ;
		}
	}

	Halfedge largest_border() 
	{
		SurfaceMeshModel::Halfedge_property<bool> is_visited = mesh->add_halfedge_property<bool>("h:visited", false);

		Halfedge result;
		int max_size = 0 ;

		foreach(Halfedge it, mesh->halfedges())
		{
			if(mesh->is_boundary(it) && !is_visited[it]) {
				int cur_size = 0 ;
				Halfedge cur = it ;
				do {
					cur_size++ ;
					is_visited[cur] = true ;
					cur = mesh->next_halfedge(cur);
				} while(cur != it) ;
				if(cur_size > max_size) {
					max_size = cur_size ;
					result = it ;
				}
			}
		}

		mesh->remove_halfedge_property(is_visited);

		return result;
	}

	void principal_axes( Halfedge h, Vec3d& center, Vec3d& v1, Vec3d& v2  ) 
	{
		// Collect border points
		std::vector<Vec3d> border_pnts;
		Halfedge cur = h ;
		do {
			Vec3d p = points[mesh->to_vertex(cur)];
			normalize_uv(p) ;
			border_pnts.push_back(p);
			cur = mesh->next_halfedge(cur);
		} while(cur != h);

		PCA3 pca( border_pnts );
		std::vector<Vec3d> axis = pca.eigenvectors();

		center = pca.center();
		v1 = axis[0];
		v2 = axis[1];
	}

	void get_border_extrema( Halfedge h, const Vec3d& center, const Vec3d& V, Vertex& vx_min, Vertex& vx_max) 
	{
		Halfedge cur = h;
		double v_min =  DBL_MAX ;
		double v_max = -DBL_MAX ;
		do {
			Vec3d p = points[mesh->to_vertex(cur)];
			normalize_uv(p) ;
			double v = dot(p - center, V) ;

			if(v < v_min) {
				v_min = v ;
				vx_min = mesh->to_vertex(cur) ;
			}

			if(v > v_max) {
				v_max = v ;
				vx_max = mesh->to_vertex(cur) ;
			}

			cur = mesh->next_halfedge(cur) ;
		} while(cur != h) ;

		tex_coord[vx_min] = Vec2d(0,0);
		tex_coord[vx_max] = Vec2d(0,1);

		is_locked[vx_min] = true;
		is_locked[vx_max] = true;
	}

	void normalize_uv(Vec3d& p){
		Vec3d v = 1.0 / radius_ * (p - center_) ;
		p = Vec3d(v.x(), v.y(), v.z()) ;
	}

	// Data
	SurfaceMeshModel * mesh;
	Vector3VertexProperty points;
	Vector3VertexProperty uv;
	BoolVertexProperty is_locked;
	int num_locked;

	// Output
	SurfaceMeshModel::Vertex_property<Vec2d> tex_coord;

	// Linear system
	Eigen::SparseMatrix<double> Mf, Mp;
	Eigen::SparseMatrix<double> A, At;
	Eigen::VectorXd U;
	Eigen::VectorXd b;
	Eigen::VectorXd x;

	std::map<int,int> v_idx;

	Vec3d center_;
	Scalar radius_;
	Vertex vx_min, vx_max;

	Vec3d V1,V2;
	Vec3d center;
};
