#pragma once

#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

#include "../surfacemesh_filter_geoheat/GeoHeatHelper.h"

struct BoundaryFitting{

	BoundaryFitting(SurfaceMeshModel * useMesh)
	{
		this->mesh = useMesh;
		this->dists = mesh->vertex_property<Scalar>("v:bf_dists",1.0);
		Vector3VertexProperty points = mesh->vertex_property<Vector3>(VPOINT);

		// Tag boundary as source points
		QSet<Vertex> boundary_src; 
		foreach(Vertex v, mesh->vertices()) if(mesh->is_boundary(v)) boundary_src.insert(v);

		// Collect vertices at boundary vector
		QVector<Vertex> boundary_verts;
		SurfaceMeshModel::Halfedge_iterator h (mesh->halfedge(boundary_src.values().front()),mesh), hend = h;
		do { 
			boundary_verts.push_back( mesh->to_vertex(h) ); 
			h = mesh->next_halfedge(h);
		} while (h != hend);

		// Four best corners
		QSet<Vertex> corners;
		{
			// Order by corner score
			GeoHeatHelper geoDist(mesh);
			ScalarVertexProperty dists_boundry = geoDist.getUniformDistance(boundary_src);

			typedef std::pair<double,Vertex> DistVert;
			std::vector<DistVert> distVert;
			foreach(Vertex v, boundary_verts)
				distVert.push_back( std::make_pair( dists_boundry[v], v ) );
			std::sort(distVert.begin(),distVert.end());

			// Filter
			for(int i = 0; i < (int)distVert.size(); i++)
			{
				Vertex & v = distVert[i].second;
				if(dists_boundry[v] == 1.0){
					distVert[i].first = 1.0;
					continue;
				}

				QSet<Vertex> k_rings;
				int k = 5;
				collectRing(v, k_rings, k);

				foreach(Vertex vv, k_rings){
					if(v == vv) continue;
					dists_boundry[vv] = 1.0;
				}
			}
		
			std::sort(distVert.begin(),distVert.end());

			for(int i = 0; i < 4; i++){
				Vertex & v = distVert[i].second;
				corners.insert(v);
			}
		}

		// Order clock-wise by traveling along boundary
		foreach(Vertex v, boundary_verts){
			if(corners.contains(v))	sorted_corners.push_back(v);
		}

		Vertex start = sorted_corners[0];
		Vertex end = sorted_corners[1];

		// Compute iso-lines from an edge
		{
			QSet<Vertex> edge_src;
			Halfedge h = mesh->halfedge(start);
			while(h != mesh->halfedge(end)){
				edge_src.insert(mesh->to_vertex(h));
				h = mesh->next_halfedge(h);
			}

			GeoHeatHelper geoDist(mesh);
			ScalarVertexProperty dists_edge = geoDist.getUniformDistance(edge_src);

			int segments = 10;

			foreach(Vertex v, mesh->vertices())
			{
				dists[v] = cos(dists_edge[v] * (segments - 1) * 2 * M_PI);
			}
		}

	}

	void collectRing( Vertex v, QSet<Vertex> & set, int level ){
		if(level > 0){
			SurfaceMeshModel::Vertex_around_vertex_circulator vi(mesh, v), vend = vi;
			do{	collectRing(vi, set, level - 1); set.insert(vi); } while(++vi != vend);
		}
		else set.insert(v);
	}

	SurfaceMeshModel* mesh;
	ScalarVertexProperty dists;
	
	// Output
	std::vector<Vertex> sorted_corners;
};
