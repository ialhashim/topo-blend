#pragma once

#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

#include "../surfacemesh_filter_geoheat/GeoHeatHelper.h"

struct BoundaryFitting : public SurfaceMeshHelper{

	BoundaryFitting(SurfaceMeshModel * useMesh, double resolution) : SurfaceMeshHelper(useMesh)
	{
		points  = this->getVector3VertexProperty(VPOINT);
		fnormal = this->computeFaceNormals();
		farea   = this->computeFaceAreas();
		elenght = this->computeEdgeLengths();

		this->dists = mesh->vertex_property<Scalar>("v:bf_dists",1.0);
		ScalarVertexProperty dists_boundry_copy = mesh->vertex_property<Scalar>("v:bf_dists_boundry_copy",0.0);

		// Collect vertices at boundary vector
		QVector<Vertex> boundary_verts;
		foreach(Edge e, mesh->edges())
		{
			// Get first half edge on boundary
			Halfedge startH = mesh->halfedge(e,0);
			if(!mesh->is_boundary(startH)) startH = mesh->halfedge(e,1);
			if(!mesh->is_boundary(startH)) continue;

			// Go along boundary
			Halfedge h = startH;
			do {
				boundary_verts.push_back(mesh->to_vertex(h));
				h = mesh->next_halfedge(h);
			} while( h != startH );

			break;
		}

		// Four best corners
		QSet<Vertex> corners;
		{
			typedef std::pair<double,Vertex> DistVert;
			std::vector<DistVert> distVert;

			// Compute distance from boundary, sorted by value
			{
				// Tag boundary as source points
				QSet<Vertex> boundary_src; 
				foreach(Vertex v, boundary_verts) boundary_src.insert(v);

				GeoHeatHelper geoDist(mesh);
				ScalarVertexProperty dists_boundry = geoDist.getUniformDistance(boundary_src);

				foreach(Vertex v, boundary_verts)
					distVert.push_back( std::make_pair( dists_boundry[v], v ) );

				std::sort(distVert.begin(),distVert.end());

				foreach(Vertex v, mesh->vertices()){
					double val = (dists_boundry[v] >= 0.10) ? 1.0 : dists_boundry[v];
					dists[v] = val;
				}

				normalize(dists);

				foreach(Vertex v, mesh->vertices())
					dists_boundry_copy[v] = dists[v];
			}

			// First corner is lowest value, second corner is furthest
			Vertex furthestVertex = boundary_verts.front();
			{
				Vertex firstCorner = distVert.front().second;
				QSet<Vertex> firstCornerSet;
				firstCornerSet.insert( firstCorner );
				corners.insert( firstCorner );

				GeoHeatHelper geoDist(mesh);
				ScalarVertexProperty dists_corner = geoDist.getUniformDistance( firstCornerSet );

				double maxDist = -DBL_MIN;
				foreach(Vertex v, boundary_verts)
				{
					double dist = dists_corner[v];
					if(dist > maxDist)
					{
						maxDist = dist;
						furthestVertex = v;
					}
				}

				corners.insert( furthestVertex );
			}

			// Third and fourth corners
			{
				// Compute weights for remaining corners
				GeoHeatHelper geoDist(mesh);
				ScalarVertexProperty dists_diag = geoDist.getUniformDistance( corners );
				foreach(Vertex v, mesh->vertices())
					if(!mesh->is_boundary(v)) dists[v] = 1.0;

				foreach(Vertex v, boundary_verts)
				{
					double weight = (1-dists[v]) * dists_diag[v];
					dists[v] = 1-(weight * (1-dists_boundry_copy[v]));
				}

				// Make sure we don't pick already picked
				foreach(Vertex v, corners)
					dists[v] = 1.0;

				// Sort scores
				distVert.clear();
				foreach(Vertex v, boundary_verts) distVert.push_back( std::make_pair( dists[v], v ) );
				std::sort(distVert.begin(),distVert.end());

				// Filter neighborhood
				for(int i = 0; i < (int)distVert.size(); i++)
				{
					Vertex v0 = distVert[i].second;
					if(dists[v0] == 1.0) continue;

					int k = 5;
					QSet<Vertex> neigh;
					collectRing(v0, neigh, k);

					foreach(Vertex v, neigh){
						if(v == v0) continue;
						dists[v] = 1.0;
					}
				}

				// Add a boundary angle weight ?
				//dists[v] *= (abs(boundaryAngle(v)) / M_PI);

				// Sort again
				distVert.clear();
				foreach(Vertex v, boundary_verts) distVert.push_back( std::make_pair( dists[v], v ) );
				std::sort(distVert.begin(),distVert.end());

				// Pick top two
				for(int i = 0; i < 2; i++){
					Vertex & v = distVert[i].second;
					corners.insert(v);
				}
			}
		}

		foreach(Vertex v, corners)
			debugPoints.push_back(points[v]);

		normalize(dists);

		// Order clock-wise by traveling along boundary
		foreach(Vertex v, boundary_verts){
			if(corners.contains(v))	sorted_corners.push_back(v);
		}

		// Estimate segment counts by an approximation of length and width of rect:
		double length = 0, width = 0;
		{
			// Length [0] -- [1]
			Vertex start = sorted_corners[0];
			Vertex end = sorted_corners[1];
			Halfedge h = mesh->halfedge(start);
			while(h != mesh->halfedge(end)){
				length += elenght[mesh->edge(h)];
				h = mesh->next_halfedge(h);
			}

			// Width [1] -- [2] - seem bad assumption for non-rectangular boundaries
			start = sorted_corners[1];
			end = sorted_corners[2];
			h = mesh->halfedge(start);
			while(h != mesh->halfedge(end)){
				width += elenght[mesh->edge(h)];
				h = mesh->next_halfedge(h);
			}
		}

		int nU = width / resolution;
		int nV = length / resolution;

		// Compute iso-lines from opposing edge
		{
			QSet<Vertex> edge_src;
			Halfedge h;

			// Side A
			Vertex start = sorted_corners[0];
			Vertex end = sorted_corners[1];

			h = mesh->halfedge(start);
			while(h != mesh->halfedge(end)){
				edge_src.insert(mesh->from_vertex(h));
				h = mesh->next_halfedge(h);
			}
			edge_src.insert(mesh->from_vertex(h));

			// Side B
			//start = sorted_corners[2];
			//end = sorted_corners[3];

			//h = mesh->halfedge(start);
			//while(h != mesh->halfedge(end)){
			//	edge_src.insert(mesh->from_vertex(h));
			//	h = mesh->next_halfedge(h);
			//}
			//edge_src.insert(mesh->from_vertex(h));

			GeoHeatHelper geoDist(mesh);
			ScalarVertexProperty dists_edge = geoDist.getUniformDistance(edge_src);

			foreach(Vertex v, mesh->vertices())
				dists[v] = dists_edge[v];

			double delta = 1.0 / (nU / 2);

			// Extract curves (iso-lines) along a remaining edge
			start = sorted_corners[1];
			end = sorted_corners[2];

			double val = 0;

			h = mesh->halfedge(start);

			while(h != mesh->halfedge(end))
			{
				Edge e = mesh->edge(h);

				double v1 = dists[mesh->vertex(e, 0)];
				double v2 = dists[mesh->vertex(e, 1)];
				if(v1 > v2)	std::swap(v1,v2);

				// This helps us deal with values near 0.0 or 1.0 when using two sides
				//double val = (num_segments - lines.size()) * delta;
				//if((int)lines.size() < num_segments * 0.5)
				//	val = lines.size() * delta;
				//if(val == 0.0) val = 0.5 * delta; 
				//if(val == 1.0) val -= 0.5 * delta;
				//
				val = floor(v2 / delta) * delta;

				if( between(val, v1, v2) )
				{
					std::vector<Vec3d> line;

					Halfedge curEdge = mesh->opposite_halfedge(h);

					QStack<Halfedge> toVisit;
					toVisit.push(curEdge);

					QSet<int> seenEdges;

					while(!toVisit.empty())
					{
						curEdge = toVisit.pop();

						if(seenEdges.contains(mesh->edge(curEdge).idx())) continue;
						seenEdges.insert(mesh->edge(curEdge).idx());

						// Go over half edges of a face
						Face startFace = mesh->face(curEdge);
						if(!mesh->is_valid(startFace))
							startFace = mesh->face(mesh->opposite_halfedge(curEdge));

						Surface_mesh::Halfedge_around_face_circulator hj(mesh, startFace), hend = hj;
						do {
							Halfedge nextEdge = mesh->opposite_halfedge(hj);

							// Add if its a value crossing edge
							double val1 = dists[mesh->to_vertex(hj)];
							double val2 = dists[mesh->from_vertex(hj)];
							if(val1 > val2)	std::swap(val1,val2);
							if( between(val, val1, val2) ) 
							{
								toVisit.push( nextEdge );
							}
						} while (++hj != hend);

						Vertex p1 = mesh->to_vertex(curEdge);
						Vertex p2 = mesh->from_vertex(curEdge);

						double b1 = dists[p1];
						double b2 = dists[p2];

						if(b1 > b2){
							std::swap(p1,p2);
							std::swap(b1,b2);
						}

						double alpha = qMin(1.0, qMax(0.0, (val - b1) / (b2 - b1) ) );

						Vec3d pnt = ((1-alpha) * points[p1]) + (alpha * points[p2]);
						line.push_back( pnt );
					}

					lines.push_back(line); 
				}

				h = mesh->next_halfedge(h);
			}
		} // end of: iso-lines extraction

		for(int i = 0; i < (int)lines.size(); i++)
			lines[i] = equidistLine(lines[i], nV);

		// clean up lines
		Array2D_Vector3 final_lines;
		foreach(Array1D_Vector3 line, lines){
			if(line.empty()) continue;

			double lineWidth = 0.0;
			for(int i = 1; i < (int)line.size(); i++)
				lineWidth += (line[i] - line[i-1]).norm();

			if(lineWidth > 1.5 * resolution)
				final_lines.push_back(line);
		}

		lines = final_lines;
	}

	std::vector<Vec3d> equidistLine(std::vector<Vec3d> & l, int numSegments)
	{
		std::vector<Vec3d> resampled_line;

		// check 
		numSegments = qMax(numSegments, 2);

		// Parameterize line by finding segments
		double totalLineLength = 0.0;

		typedef std::pair<double,int> LengthSegment;
		std::vector< LengthSegment > segments;

		// Add a start segment
		segments.push_back(LengthSegment(totalLineLength, 1));

		for(int i = 1; i < (int)l.size(); i++)
		{
			double curLen = (l[i] - l[i-1]).norm();
			totalLineLength += curLen;
			segments.push_back( LengthSegment(totalLineLength, i) );
		}

		if(totalLineLength == 0.0)
			return resampled_line;

		// Re-sample line
		for(int i = 0; i < numSegments; i++)
		{
			double t = totalLineLength * (double(i) / (numSegments-1));

			std::vector< LengthSegment >::iterator it = lower_bound(segments.begin(), 
				segments.end(), LengthSegment(qMin(t, totalLineLength), -1));

			// Find start
			int idx = it->second;

			double seg_range = segments[idx].first - segments[idx-1].first;
			double seg_start = segments[idx-1].first;

			double alpha = (t - seg_start) / seg_range;

			resampled_line.push_back( ( (1-alpha) * l[idx-1] ) + (alpha * l[idx]) );
		}

		return resampled_line;
	}

	bool between(double & val, double minVal, double maxVal) 
	{
		return ((val-minVal) >= 0) && ((val-maxVal) <= 0);
	}

	double boundaryAngle( Vertex v )
	{
		std::vector<Vertex> vj;

		foreach(Halfedge h, mesh->onering_hedges(v)){
			if(mesh->is_boundary(mesh->edge(h)))
				vj.push_back(mesh->to_vertex(h));
		}

		Vec3d a = (points[vj[0]] - points[v]).normalized();
		Vec3d b = (points[vj[1]] - points[v]).normalized();

		return dot(a,b);
	}

	void normalize( ScalarVertexProperty & vprop )
	{
		// Get range
		double min_d = DBL_MAX, max_d = -min_d;
		foreach(Vertex v, mesh->vertices()){
			min_d = qMin(vprop[v], min_d);
			max_d = qMax(vprop[v], max_d);
		}
		double range = max_d - min_d;

		foreach(Vertex v, mesh->vertices()){
			vprop[v] = (vprop[v] - min_d) / range;
		}
	}

	void collectRing( Vertex v, QSet<Vertex> & set, int level ){
		if(level > 0){
			SurfaceMeshModel::Vertex_around_vertex_circulator vi(mesh, v), vend = vi;
			do{	collectRing(vi, set, level - 1); set.insert(vi); } while(++vi != vend);
		}
		else set.insert(v);
	}

	ScalarVertexProperty dists;

	// Output
	std::vector<Vertex> sorted_corners;
	std::vector< std::vector<Vec3d> > lines;

	std::vector< std::vector<Vec3d> > controlPoints;

	// DEBUG:
	std::vector<Vec3d> debugPoints;
};
