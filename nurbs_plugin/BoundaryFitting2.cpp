#include <set>
#include "BoundaryFitting2.h"

#include "../surfacemesh_filter_geoheat/GeoHeatHelper.h"
#include "PCA.h"
#include "LineSegment.h"

using namespace SurfaceMesh;
using namespace NURBS;

#include "BoundaryFittingGlobal.h"

BoundaryFitting2::BoundaryFitting2( SurfaceMeshModel * mesh )
{
	part = mesh;

	vals = part->vertex_property<Scalar>("v:vals", 0);
	boundaryCurv = part->vertex_property<Scalar>("v:vivo", 0);
	dists = part->vertex_property<Scalar>("v:bf_dists", 0);

	fgradient = part->face_property<Vector3> ("f:fitting_gradient", Vector3(0));

	part->update_face_normals();
	part->update_vertex_normals();
	normals = part->vertex_property<Vector3>(VNORMAL);
	points = part->vertex_property<Vector3>(VPOINT);

	foreach(Vertex v, part->vertices()) tree.addPoint(points[v]);
	tree.build();

	doFit();
}

void BoundaryFitting2::doFit()
{
	QVector<Vertex> boundry;
	foreach(Vertex v, boundaryVerts()) boundry.push_back(v);

	int range = boundry.size() * 0.05;

	// Compute distance from boundry to use in gradient computation
	{
		GeoHeatHelper geoDist(part);
		QSet<Vertex> boundSet;foreach(Vertex v,boundry)boundSet.insert(v);
		ScalarVertexProperty dists_boundry = geoDist.getUniformDistance(boundSet);
		normalize(dists_boundry);
		//foreach(Vertex v, part->vertices()){
		//	std::vector<Vertex> n = collectRings(v, 12);
		//	double sum = 0.0;
		//	int usedCount = 0;
		//	foreach(Vertex vj, n){
		//		if(!part->is_boundary(vj)){
		//			sum += dists_boundry[vj];
		//			usedCount++;
		//		}
		//	}
		//	vals[v] = sum / usedCount;
		//}
		foreach(Vertex v, part->vertices()) vals[v] = dists_boundry[v];
		normalize(vals);
	}

	// Compute face gradients using values computed above
	{
		gradientFaces();
	}

	{
		double cVal = 0;
		double minVal = DBL_MAX;

		// Filter for near corner cases
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			QVector<Vertex> nei = neighbours(i,range,boundry);
			double sum = 0;

			for(int j = 0; j < (range * 0.5); j++){
				Vector3 e1 = (points[nei[j]] - points[nei[j+1]]).normalized();
				Vector3 e2 = (points[nei[nei.size()-1 - j]] - points[nei[nei.size()-2 - j]]).normalized();

				double angle = abs(signedAngle(e1,e2,normals[boundry[j]]));
				sum += angle;
			}

			cVal = sum / (range * 0.5);
			boundaryCurv[boundry[i]] = cVal;

			minVal = qMin(minVal,cVal);
		}

		foreach(Vertex v, part->vertices()) 
			if(!part->is_boundary(v)) boundaryCurv[v] = minVal;

		normalize(boundaryCurv);
	}

	// Cluster boundary
	{
		vdirection = part->vertex_property<Vector3>("v:direction",Vector3(0));
		Vector3VertexProperty vnew = part->vertex_property<Vector3>("v:directionNew",Vector3(0));

		// Assign directions from adjacent faces
		foreach( Vertex v, part->vertices() ){
			std::vector<Face> adjF;
			foreach(Halfedge h, part->onering_hedges(v)){
				Face f = part->face(h);
				if(!part->is_valid(f)) continue;
				adjF.push_back(f);
			}

			Vector3 sum(0);
			foreach(Face f, adjF) sum += fgradient[f];
			vdirection[v] = sum / adjF.size();
		}

		// Realign boundary vectors
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			Vertex v = boundry[i];
			if(boundaryCurv[v] > 0.8) continue;

			Vec3d edge = (points[boundry[(i+1)%boundry.size()]] - points[v]).normalized();

			Vec3d prev = vdirection[v];
			vdirection[v] = prev.norm() * cross(normals[v], edge);
		}

		// Smooth vectors
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			Vertex prev = boundry[PREV(i,boundry.size())];
			Vertex next = boundry[NEXT(i,boundry.size())];

			vnew[boundry[i]] = (vdirection[prev] + vdirection[next]) * 0.5;
		}
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			vdirection[boundry[i]] = vnew[boundry[i]];
		}

		// Compare with closest central points
		typedef QPair<Vector3,Vector3> QPairVector3;
		QVector< QPairVector3 > pointNormal;
		foreach(Vertex v, part->vertices())
		{
			if(vals[v] > 0.9){
				pointNormal.push_back(qMakePair(points[v],normals[v]));
			}
		}

		// Get a reasonable axis
		std::vector<Vec3d> allPoints;
		foreach(Vertex v, part->vertices()) allPoints.push_back(points[v]);
		Vector3 first, second, third;
		PCA::mainAxis(allPoints,first,second,third);
		Vector3 axis = first.normalized();

		typedef QMap< int, QVector<int> > GroupAssignment;
		typedef QPair< std::map<int,Vector3>, GroupAssignment > DirectionsGroups;
		QMap< double, DirectionsGroups > errorMap;

		for(int i = 0; i < (int)boundry.size(); i++)
		{
			Vertex startVertex = boundry[i];

			// Relative to some boundary vertex
			Vector3 mainDirection = vdirection[startVertex].normalized();
			Vector3 mainNormal = normals[startVertex];

			// Preset directions
			std::map<int,Vector3> d;
			d[0] = mainDirection;
			d[1] = rotatedVec(mainDirection, M_PI_2, mainNormal);
			d[2] = -mainDirection;
			d[3] = -d[1];

			QMap< int, int > groups;

			for(int i = 0; i < (int)boundry.size(); i++)
			{
				Vertex v = boundry[i];

				double minDist = DBL_MAX;
				foreach(QPairVector3 pn, pointNormal){
					Vector3 p = pn.first;
					Vector3 n = pn.second;
					double dist = (points[v] - p).norm();
					if(dist < minDist){
						minDist = dist;
						axis = n;
					}
				}

				double angle = signedAngle(mainDirection,vdirection[v].normalized(),axis);

				// Classify by angle
				if(abs(angle) > M_PI_4){
					if(abs(angle) < (M_PI - M_PI_4)){
						if(angle > 0)
							groups[v.idx()] = 1;
						else
							groups[v.idx()] = 3;
					}
					else
						groups[v.idx()] = 2;
				}
				else
				{
					groups[v.idx()] = 0;  // default
				}
			}

			GroupAssignment grp;
			for(int i = 0; i < (int)boundry.size(); i++){
				int vid = boundry[i].idx();
				grp[groups[vid]].push_back(vid);
			}

			// Compute total error at this vertex
			double total_error = 0;
			QMap<int,Vector3> curDirections;

			// Average
			QMap< int, Vector3 > grpAvg;
			foreach( int id, grp.keys() ){
				QVector<int> items = grp[id];

				// Compute average
				Vector3 avg(0);
				foreach(int vidx, items) avg += vdirection[Vertex(vidx)];
				avg /= items.size();

				foreach(int vidx, items){
					double angle = acos(qRanged(-1.0, dot(avg, vdirection[Vertex(vidx)]), 1.0));
					total_error += angle;
				}
			}

			errorMap[total_error] = qMakePair(d,grp);

		} // -- END OF SEARCH

		// Result of search
		double lowError = errorMap.keys().front();
		DirectionsGroups dg = errorMap[ lowError ];
		std::map<int,Vector3> directions = dg.first;
		GroupAssignment grp = dg.second;
		QMap<int,int> itemGroup;

		foreach(int gid, grp.keys()){
			QVector<int> items = grp[gid];

			foreach(int vidx, items){
				Vertex v(vidx);
				vals[v] = double(gid) / 3;
				itemGroup[v.idx()] = gid;
				//vdirection[v] = directions[gid];
			}
		}

		// Sort inside the groups based on boundary
		std::rotate(boundry.begin(),boundry.begin() + boundry.indexOf(Vertex(grp[grp.keys().front()].front())),boundry.end());
		QMap< int, QVector<int> > sortedGroups;
		foreach(Vertex v, boundry)
			sortedGroups[itemGroup[v.idx()]].push_back(v.idx());

		// Shortest path
		{
			Vertex fromVert = Vertex(sortedGroups[sortedGroups.keys().at(0)].front());
			Vertex toVert = Vertex(sortedGroups[sortedGroups.keys().at(2)].front());

			foreach(Vector3 p, geodesicPath(points[ fromVert ], points[ toVert ]))
				debugPoints.push_back(p);

			QSet<Vertex> vertSet;

			//foreach(int vidx, grp[grp.keys().at(0)]) vertSet.insert( Vertex(vidx) );
			//foreach(int vidx, grp[grp.keys().at(2)]) vertSet.insert( Vertex(vidx) );

			vertSet.insert(fromVert);

			GeoHeatHelper geoDist(part);
			ScalarVertexProperty dists_edges = geoDist.getUniformDistance( vertSet );
			foreach(Vertex v, geoDist.shortestVertexPath( toVert )) dists[v] = 1;
			normalize(dists);
		}


		//// Reassign
		//for(int i = 0; i < (int)boundry.size(); i++)
		//{
		//	Vertex v = boundry[i];
		//	vdirection[v] = vnew[v];
		//}

		part->remove_vertex_property(vnew);
	}
}

std::vector<Vec3d> BoundaryFitting2::geodesicPath(Vec3d fromPoint, Vec3d toPoint)
{
	std::vector<Vec3d> path;

	typedef QPair<Halfedge,double> HalfedgeTime;
	typedef QMap<double, HalfedgeTime > DistanceHalfedge;

	Vertex end(tree.closest(toPoint));
	Vertex start(tree.closest(fromPoint));
	QSet<Vertex> vertSet; vertSet.insert(end);
	Halfedge h = part->halfedge(start);

	GeoHeatHelper geoDist(part);
	ScalarVertexProperty d = geoDist.getUniformDistance( vertSet );

	path.push_back( fromPoint );

	// Start with best face
	QMap<double, Face> faces;
	foreach(Halfedge he, part->onering_hedges(start)){
		Face f = part->face(he);
		if(!f.is_valid()) continue;

		double minValue = DBL_MAX;
		Vertex minVert;
		Surface_mesh::Vertex_around_face_circulator vit = part->vertices(f),vend=vit;
		do{ if(d[vit] < minValue) { minValue = d[vit]; minVert = vit; } } while(++vit != vend);
		faces[ d[minVert] ] = f;
	}

	Face f = faces.values().front();

	while( true )
	{
		double dist_to_target = (path.back() - toPoint).norm();
		if(dist_to_target == 0) break;

		DistanceHalfedge edgeDists;

		// Check edges of current face
		Surface_mesh::Halfedge_around_face_circulator hj(part, f), hend = hj;
		do{ 
			Vertex a = part->from_vertex(hj);
			Vertex b = part->to_vertex(hj);

			Line l(points[a], points[b]);

			double t = l.timeAt( path.back() );
			double d_value = ((1-t) * d[a]) + (t * d[b]);
			edgeDists[ d_value ] = qMakePair((Halfedge)hj,t);
		} while (++hj != hend);
			
		// Get best edge, and path point at edge
		HalfedgeTime bestEdgeTime = edgeDists.values().front();
		Halfedge h = bestEdgeTime.first;
		double t = bestEdgeTime.second;
		Vertex a = part->from_vertex(h);
		Vertex b = part->to_vertex(h);
		Vector3 bestPointOnEdge = ((1-t) * points[a]) + (t * points[b]);

		path.push_back( bestPointOnEdge );

		// Move to next face
		f = part->face( part->opposite_halfedge(h) );

		if(!part->is_valid(f)) break;
	}

	return path;
}

std::vector<SurfaceMesh::Vertex> BoundaryFitting2::collectRings( SurfaceMesh::Vertex v, size_t min_nb, bool isBoundary )
{
	std::vector<Vertex> all;
	std::vector<Vertex> current_ring, next_ring;
	SurfaceMeshModel::Vertex_property<int> visited_map = part->vertex_property<int>("v:visit_map",-1);

	//initialize
	visited_map[v] = 0;
	current_ring.push_back(v);
	all.push_back(v);

	int i = 1;

	while ( (all.size() < min_nb) &&  (current_ring.size() != 0) ){
		// collect i-th ring
		std::vector<Vertex>::iterator it = current_ring.begin(), ite = current_ring.end();

		for(;it != ite; it++){
			// push neighbors of 
			SurfaceMeshModel::Halfedge_around_vertex_circulator hedgeb = part->halfedges(*it), hedgee = hedgeb;
			do{
				Vertex vj = part->to_vertex(hedgeb);

				if(isBoundary){
					if(!part->is_boundary(vj)) continue;
				}

				if (visited_map[vj] == -1){
					visited_map[vj] = i;
					next_ring.push_back(vj);
					all.push_back(vj);
				}

				++hedgeb;
			} while(hedgeb != hedgee);
		}

		//next round must be launched from p_next_ring...
		current_ring = next_ring;
		next_ring.clear();

		i++;
	}

	//clean up
	part->remove_vertex_property(visited_map);

	return all;
}

void BoundaryFitting2::normalize( ScalarVertexProperty & vprop )
{
	// Get range
	double min_d = DBL_MAX, max_d = -min_d;
	foreach(Vertex v, part->vertices()){
		min_d = qMin(vprop[v], min_d);
		max_d = qMax(vprop[v], max_d);
	}
	double range = max_d - min_d;

	foreach(Vertex v, part->vertices()){
		vprop[v] = (vprop[v] - min_d) / range;
	}
}

QVector<Vertex> BoundaryFitting2::boundaryVerts()
{
	// Collect vertices at boundary vector
	QVector<Vertex> boundary_verts;
	foreach(Edge e, part->edges())
	{
		// Get first half edge on boundary
		Halfedge startH = part->halfedge(e,0);
		if(!part->is_boundary(startH)) startH = part->halfedge(e,1);
		if(!part->is_boundary(startH)) continue;

		// Go along boundary
		Halfedge h = startH;
		do {
			boundary_verts.push_back(part->to_vertex(h));
			h = part->next_halfedge(h);
		} while( h != startH );

		break;
	}
	return boundary_verts;
}

void BoundaryFitting2::gradientFaces( bool isNormalizeNegateGradient )
{
	SurfaceMeshHelper h(part);
	Vector3FaceProperty fnormal = h.computeFaceNormals();
	ScalarFaceProperty farea = h.computeFaceAreas();
	Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

	// Compute gradient on faces
	foreach(Face f, part->faces()){
		Vector3 vsum(0);

		Surface_mesh::Halfedge_around_face_circulator h(part, f), hend = h;
		do{
			Vector3 ei = points[part->from_vertex(h)] - points[part->from_vertex(part->prev_halfedge(h))];
			Vertex i = part->to_vertex(h);
			vsum += vals[i] * cross(fnormal[f], ei);
		}while (++h != hend);

		fgradient[f] = ( 1.0 / (2.0 * farea[f]) ) * vsum;

		if(isNormalizeNegateGradient)
			fgradient[f] = (-fgradient[f]).normalized();
	}

	// Smooth gradient
	for(int itr = 0; itr < 10; itr++)
	{
		std::vector<Vec3d> newGrad(part->n_faces(),Vector3(0));
		std::vector<Vec3d> avgNormal(part->n_faces(),Vector3(0));

		foreach(Face f, part->faces())
		{
			// Collect neighboring faces
			std::set<int> face_ids;

			Surface_mesh::Vertex_around_face_circulator vit = part->vertices(f),vend=vit;
			do{ 
				foreach(Halfedge h, part->onering_hedges(vit)){
					Face curFace = part->face(h);
					if(part->is_valid(curFace) && !part->is_boundary(curFace))	
					{
						face_ids.insert( curFace.idx() );
					}
				}
			} while(++vit != vend);

			// Average gradients
			foreach(int fid, face_ids){
				newGrad[f.idx()] += fgradient[Face(fid)];
				avgNormal[f.idx()] += fnormal[Face(fid)];
			}
			newGrad[f.idx()] /= face_ids.size();
			avgNormal[f.idx()] /= face_ids.size();
		}

		// Assign new values
		foreach(Face f, part->faces())
		{
			fnormal[f] = avgNormal[f.idx()];

			// Project on plane defined by face normal
			Scalar t = dot(fnormal[f], newGrad[f.idx()]);
			fgradient[f] =  newGrad[f.idx()] - t * fnormal[f];

			//fgradient[f] = newGrad[f.idx()];
		}
	}
}

QVector<SurfaceMesh::Vertex> BoundaryFitting2::neighbours( int start, int range, QVector<SurfaceMesh::Vertex> all )
{
	QVector<SurfaceMesh::Vertex> result;

	range = qMin(range, (int)(all.size() * 0.5));

	int halfRange = (range * 0.5);

	for(int i = -halfRange; i < halfRange; i++)
	{
		int idx = start + i;
		if(idx < 0) idx = all.size() - abs(idx);
		idx = idx % all.size();
		result.push_back(all[idx]);
	}

	return result;
}

SurfaceMesh::Halfedge BoundaryFitting2::get_halfedge( Vertex start, Vertex end )
{
	Halfedge h = part->halfedge(start);
	const Halfedge hh = h;
	if (h.is_valid()){
		do{
			if (part->to_vertex(h) == end) return h;
			h = part->cw_rotated_halfedge(h);
		} while (h != hh);
	}
	return SurfaceMesh::Halfedge();
}
