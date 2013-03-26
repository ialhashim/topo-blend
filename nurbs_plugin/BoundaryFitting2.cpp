#include <set>
#include "BoundaryFitting2.h"

#include "../surfacemesh_filter_geoheat/GeoHeatHelper.h"
#include "PCA.h"
#include "LineSegment.h"

using namespace SurfaceMesh;
using namespace NURBS;

#include "BoundaryFittingGlobal.h"

BoundaryFitting2::BoundaryFitting2( SurfaceMeshModel * mesh, int numSegments, int numSegmentsV )
{
	this->part = mesh;
	this->segments = numSegments;
	this->segmentsV = numSegmentsV;

	vals = part->vertex_property<Scalar>("v:vals", 0);
	boundaryCurv = part->vertex_property<Scalar>("v:vivo", 0);
	dists = part->vertex_property<Scalar>("v:bf_dists", 0);

	fgradient = part->face_property<Vector3> ("f:fitting_gradient", Vector3(0));
	vgradient = part->vertex_property<Vector3> ("v:fitting_gradient", Vector3(0));

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
		foreach(Vertex v, part->vertices()) vals[v] = dists_boundry[v];
		normalize(vals);
	}

	// Compute face gradients using values computed above
	{
		gradientFaces( vals );
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

		// Smooth vectors
		for(int itr = 0; itr < 2; itr++)
		{
			for(int i = 0; i < (int)boundry.size(); i++){
				Vertex prev = boundry[PREV(i,boundry.size())];
				Vertex next = boundry[NEXT(i,boundry.size())];

				vnew[boundry[i]] = (vdirection[prev] + vdirection[next]) * 0.5;
			}
			for(int i = 0; i < (int)boundry.size(); i++){
				vdirection[boundry[i]] = vnew[boundry[i]];
			}
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
		QMap<int,int> itemGroup,itemGroup2;
		 
		// Initial assignments
		foreach(int gid, grp.keys())
		{
			QVector<int> items = grp[gid];
			foreach(int vidx, items) itemGroup[vidx] = gid;
		}

		// Filter based on neighbours
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			int prev = PREV(i,boundry.size());
			int next = NEXT(i,boundry.size());

			int grpPrev = itemGroup[boundry[prev].idx()];
			int grpCur = itemGroup[boundry[i].idx()];
			int grpNext = itemGroup[boundry[next].idx()];

			QMap<int,int> grpCount;
			foreach(Vertex v, this->neighbours(i,5,boundry))
				grpCount[itemGroup[v.idx()]]++;
			
			int oldID = itemGroup[boundry[i].idx()];
			int newID = sortQMapByValue(grpCount).back().second;

			itemGroup2[boundry[i].idx()] = newID;
		}

		GroupAssignment newGrp;
		foreach(int vid, itemGroup2.keys())	newGrp[ itemGroup2[vid] ].push_back(vid);

		// New assignments
		foreach(int gid, newGrp.keys())
		{
			QVector<int> items = newGrp[gid];

			std::vector<Vec3d> edgePoints;

			foreach(int vidx, items) 
			{
				Vertex v(vidx);
				vals[v] = double(gid + 1) / 4;
				itemGroup[vidx] = gid;

				edgePoints.push_back(points[v]);
				//vdirection[v] = directions[gid];
			}

			main_edges.push_back(edgePoints);
		}

		// Sort inside the groups based on boundary
		QMap< int, QVector<int> > sortedGroups;
		int first_group_idx = 0;

		// Find a start, i.e. an item with different group as two before
		for(int i = 0; i < (int)boundry.size(); i++)
		{
			int prev1 = PREV(i,boundry.size());
			int prev2 = PREV(prev1,boundry.size());
			
			int prGrp1 = itemGroup[boundry[prev1].idx()];
			int prGrp2 = itemGroup[boundry[prev2].idx()];
			int curGrp = itemGroup[boundry[i].idx()];

			// We found a start
			if(prGrp1 == prGrp2 && prGrp2 != curGrp){
				first_group_idx = i;
				break;
			}
		}
		
		std::rotate( boundry.begin(), boundry.begin() + first_group_idx, boundry.end() );
		foreach(Vertex v, boundry)	sortedGroups[itemGroup[v.idx()]].push_back( v.idx() );

		// Something failed
		if(sortedGroups.size() < 4) return;

		QMap<int,int> countGroup;
		foreach(int gid, sortedGroups.keys()) countGroup[sortedGroups[gid].size()] = gid;

		// Shortest paths
		{
			int startGroup = 1;

			Vertex v1 = Vertex(sortedGroups[(startGroup + 0) % 4].front());
			Vertex v2 = Vertex(sortedGroups[(startGroup + 1) % 4].front());
			Vertex v3 = Vertex(sortedGroups[(startGroup + 2) % 4].front());
			Vertex v4 = Vertex(sortedGroups[(startGroup + 3) % 4].front());

			corners.push_back(points[v1]);
			corners.push_back(points[v2]);
			corners.push_back(points[v3]);
			corners.push_back(points[v4]);

			// Draw rectangle
			foreach(Vector3 p, geodesicPath( points[ v1 ], points[v2] ))	debugPoints.push_back(p);
			foreach(Vector3 p, geodesicPath( points[ v2 ], points[v3] ))	debugPoints.push_back(p);
			foreach(Vector3 p, geodesicPath( points[ v3 ], points[v4] ))	debugPoints.push_back(p);
			foreach(Vector3 p, geodesicPath( points[ v4 ], points[v1] ))	debugPoints.push_back(p);

			// Extract fitted surface
			std::vector<Vec3d> startPath, endPath;
			foreach(Vector3 p, geodesicPath(points[ v1 ], points[ v2 ]))	startPath.push_back(p);
			foreach(Vector3 p, geodesicPath(points[ v4 ], points[ v3 ]))	endPath.push_back(p);

			if(segmentsV < 0) segmentsV = segments;

			std::vector< std::vector<Vec3d> > paths = geodesicPaths(startPath, endPath, segments);
			foreach(std::vector<Vec3d> path, paths)
			{
				lines.push_back( equidistLine(path, segments) );
			}
		}

		part->remove_vertex_property(vnew);
	}
}

std::vector< std::vector<Vec3d> > BoundaryFitting2::geodesicPaths( std::vector<Vec3d> fromPoints, std::vector<Vec3d> toPoints, int segments )
{
	std::vector< std::vector<Vec3d> > paths;

	SurfaceMeshHelper helper(part);
	Vector3FaceProperty fnormal = helper.computeFaceNormals();
	Vector3FaceProperty fcenter = helper.computeFaceBarycenters();
	
	// for geodesic computation
	QVector<Vertex> vertVector;
	QSet<Vertex> vertSet; 
	foreach(Vec3d p, toPoints) vertVector.push_back(Vertex(tree.closest(p)));
	foreach(Vertex v, vertVector) vertSet.insert(v);

	// for actual segments
	if(segments > 0)
	{
		fromPoints = equidistLine(fromPoints, segments);
		toPoints = equidistLine(toPoints, segments);
	}

	//// Use a single distance map for all points
	//GeoHeatHelper geoDist(part);
	//ScalarVertexProperty d = geoDist.getUniformDistance( vertSet );
	//gradientFaces(d, false, true);

	for(int sid = 0; sid < (int)fromPoints.size(); sid++)
	{
		std::vector<Vec3d> path;

		Vertex endVertex( tree.closest(toPoints[sid]) );

		// Start with a good face
		Vector3 fromPoint = fromPoints[sid];
		Face f = getBestFace(fromPoint, Vertex(tree.closest(fromPoint))), prevF = f;

		// End with a good face
		Vector3 toPoint = toPoints[sid];
		Face endFace = getBestFace(toPoint, endVertex);

		// Single source
		QSet<Vertex> mySet; mySet.insert( endVertex );
		GeoHeatHelper geoDist(part);
		ScalarVertexProperty d = geoDist.getUniformDistance( mySet );
		gradientFaces(d, false, true);

		// Avoid spiraling paths
		if(part->has_edge_property<bool>("e:visited")) 
			part->remove_edge_property<bool>(part->edge_property<bool>("e:visited"));
		BoolEdgeProperty evisited = part->edge_property<bool>("e:visited",false);

		Halfedge bestEdge;
		double bestTime;

		// Add start point to path
		path.push_back( fromPoint );

		while( true )
		{
			Vector3 prevPoint = path.back();
			Vector3 direction = fgradient[prevF];

			//std::vector<Vec3d> vpt = trianglePoints( prevF );
			//std::vector<Vertex> vrt = triangleVertices( prevF );
			//Vec3d A=vpt[0],B=vpt[1],C=vpt[2];
			//Vector3 direction = get_barycentric(barycentric(prevPoint,A,B,C),
			//vgradient[vrt[0]],vgradient[vrt[1]],vgradient[vrt[2]]);

			// Special boundary case
			if( !f.is_valid() ) 
			{
				Vertex a = part->from_vertex(bestEdge), b = part->to_vertex(bestEdge);
				Vector3 projCenter = Line( points[a], points[b] ).project( fcenter[prevF] );
				Vector3 N = (fcenter[prevF] - projCenter).normalized();
				Vector3 Ri = direction;

				// Reflect ray with bias
				direction = ( Ri - ( N * 1.5 * dot(Ri,N) ) ).normalized();
				f = prevF;
			}
			else
			{
				// Follow face direction
				Scalar t = dot(fnormal[f], direction);
				direction = (direction - (t * fnormal[f])).normalized();
			}

			bestTime = 0.0;
			bestEdge = getBestEdge( prevPoint, direction, f, bestTime );

			// Didn't find a good edge
			if(bestTime < 0.0 || evisited[part->edge(bestEdge)]) break;

			Vertex a = part->from_vertex(bestEdge);
			Vertex b = part->to_vertex(bestEdge);
			Line edgeSegment(points[a],points[b]);
			Vector3 ipoint = edgeSegment.pointAt( qRanged(0.001, bestTime, 0.999 ) );

			// Make sure its on triangle
			std::vector<Vec3d> vp = trianglePoints(f);
			ClosestPointTriangle(ipoint,vp[0],vp[1],vp[2],ipoint);

			path.push_back( ipoint );

			evisited[part->edge(bestEdge)] = true;

			prevF = f;
			f = part->face(part->opposite_halfedge( bestEdge ));

			// Reached end face
			if(endFace == f)
			{
				//path.push_back( toPoint );
				break;
			}
		}

		paths.push_back(path);
	}

	return paths;
}

std::vector<Vec3d> BoundaryFitting2::geodesicPath(Vec3d fromPoint, Vec3d toPoint)
{
	return geodesicPaths(std::vector<Vec3d>(1,fromPoint),std::vector<Vec3d>(1,toPoint)).front();
}

SurfaceMesh::Face BoundaryFitting2::getBestFace( Vec3d & point, Vertex guess )
{
	Face best_face;
	Vector3 isect(0), bestPoint(0);
	double minDist = DBL_MAX;

	// Find closest face to point
	foreach(Vertex v, collectRings(guess, 6))
	{
		foreach(Halfedge h, part->onering_hedges(v))
		{
			Face f = part->face(h);
			if(!f.is_valid()) continue;

			std::vector<Vec3d> vp = trianglePoints( f );
			double dist = ClosestPointTriangle(point, vp[0], vp[1], vp[2], isect);

			if(dist < minDist)
			{
				best_face = f;
				minDist = dist;
				bestPoint = isect;
			}
		}
	}

	// Shrink to make sure we are inside face not on edges
	std::vector<Vec3d> vp = trianglePoints( best_face );
	Vec3d faceCenter = (vp[0] + vp[1] + vp[2]) / 3.0;
	Vec3d delta = point - faceCenter;
	point = faceCenter + (delta * 0.99);

	return best_face;
}

SurfaceMesh::Halfedge BoundaryFitting2::getBestEdge(Vec3d prevPoint, Vec3d direction, Face f, double & bestTime)
{
	QMap<double,Halfedge> projections;
	QMap<double,double> projectionsTime;

	double threshold = 1e-5;
	double bigNum = 10000;

	direction.normalize();

	Surface_mesh::Halfedge_around_face_circulator hj(part, f), hend = hj;
	do{ 
		Vertex a = part->from_vertex(hj);
		Vertex b = part->to_vertex(hj);

		Line edgeSegment(points[a],points[b]);
		Line raySegment(prevPoint, prevPoint + (direction) * bigNum );

		Vector3 p_edge,p_ray;
		edgeSegment.intersectLine(raySegment, p_edge, p_ray);

		if((p_edge - prevPoint).norm() < threshold) continue;

		double space = (p_edge - p_ray).norm();

		space -= dot(direction, (p_edge - prevPoint).normalized());

		projections[space] = hj;
		projectionsTime[space] = edgeSegment.timeAt(p_edge);

	} while (++hj != hend);

	if(projections.empty()){
		bestTime = -1;
		return Halfedge();
	}

	bestTime = projectionsTime.values().front();
	return projections.values().front();
}

std::vector<Vec3d> BoundaryFitting2::trianglePoints( Face f )
{
	std::vector<Vector3> f_vec; 
	Surface_mesh::Vertex_around_face_circulator vit = part->vertices(f),vend=vit;
	do{ f_vec.push_back(points[vit]); } while(++vit != vend);
	return f_vec;
}

std::vector<Vertex> BoundaryFitting2::triangleVertices( Face f )
{
	std::vector<Vertex> f_vec; 
	Surface_mesh::Vertex_around_face_circulator vit = part->vertices(f),vend=vit;
	do{ f_vec.push_back(vit); } while(++vit != vend);
	return f_vec;
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

void BoundaryFitting2::gradientFaces( ScalarVertexProperty & functionVal, bool isSmooth, bool isNormalizeNegateGradient)
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
			vsum += functionVal[i] * cross(fnormal[f], ei);
		}while (++h != hend);

		fgradient[f] = ( 1.0 / (2.0 * farea[f]) ) * vsum;

		if(isNormalizeNegateGradient)
			fgradient[f] = (-fgradient[f]).normalized();
	}

	if( isSmooth )
	{
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


	// Assign vertex gradient from adjacent faces
	foreach( Vertex v, part->vertices() ){
		std::vector<Face> adjF;
		foreach(Halfedge h, part->onering_hedges(v)){
			Face f = part->face(h);
			if(!part->is_valid(f)) continue;
			adjF.push_back(f);
		}
		Vector3 sum(0);
		foreach(Face f, adjF) sum += fgradient[f];
		vgradient[v] = sum / adjF.size();
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
