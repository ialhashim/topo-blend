#include "segment.h"
#include "StarlabDrawArea.h"
#include "../CustomDrawObjects.h"
#include <QStack>

#include "Graph.h"

using namespace std;
using namespace SurfaceMeshTypes;

#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )
#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)

typedef SurfaceMeshModel::Halfedge_around_face_circulator HalfedgeFaceIter;
typedef SurfaceMeshModel::Vertex_around_face_circulator VertFaceIter;
typedef SurfaceMeshModel::Face_around_vertex_circulator FaceVertIter;
typedef SurfaceMeshModel::Vertex_around_vertex_circulator VertIter;

uint qHash( const SurfaceMeshModel::Face &key ){return qHash(key.idx()); }
uint qHash( const SurfaceMeshModel::Vertex &key ){return qHash(key.idx()); }

void segment::initParameters(RichParameterSet* pars){
	pars->addParam( new RichFloat("angle_threshold", 10.0f, "Angle threshold"));
	pars->addParam( new RichFloat("min_radius_scale", 0.1f, "Minimum component scale"));
	pars->addParam( new RichInt("k_nighbours", 3, "K-levels"));
	pars->addParam( new RichBool("visualize", true, "Visualize regions"));
	pars->addParam( new RichBool("extract", false, "Extract regions"));

	// Create helper object to access points and edge lengths
	SurfaceMeshHelper h(mesh());
	points = h.getVector3VertexProperty(SurfaceMeshTypes::VPOINT);
	farea = h.computeFaceAreas();
	elen = h.computeEdgeLengths();
}

void segment::initMesh()
{
	resetClassfication();
	resetVisitFlag();
	resetVertexClass();

	faceTarget = mesh()->face_property<QString>("f:target", "");
}

void segment::resetClassfication()
{
	mesh()->remove_face_property<int>(fclass);
	fclass = mesh()->add_face_property<int>("f:class", SHEET);
}

void segment::resetVisitFlag()
{
	mesh()->remove_face_property<bool>(fvisited);
	fvisited = mesh()->add_face_property("f:visited", false);
}

void segment::resetVertexClass()
{
	mesh()->remove_vertex_property<Scalar>(vclass);
	vclass = mesh()->add_vertex_property<Scalar>("v:class", 0);
}

void segment::applyFilter(RichParameterSet* pars)
{
	drawArea()->deleteAllRenderObjects();

	// Get user specified parameters
	double theta = pars->getFloat("angle_threshold");
	double minRadiusScale = pars->getFloat("min_radius_scale");
	int k = pars->getInt("k_nighbours");
	bool visualize = pars->getBool("visualize");
	bool extract = pars->getBool("extract");

	double minRadius = minRadiusScale * mesh()->bbox().size().length();

	// Perform segmentation
	performCurveSheetSegmentation(theta, minRadius, k, extract, visualize);
}

void segment::performCurveSheetSegmentation(double theta, double minRadius, int k, bool isExtract, bool isVisualize)
{
	// Clear debug artifacts
	drawArea()->deleteAllRenderObjects();
	PolygonSoup * poly_soup = new PolygonSoup;
	LineSegments * lines = new LineSegments;
	PointSoup * point_soup = new PointSoup;

	initMesh();

	// 1) Check valid faces by minimum angle
	foreach(Face f, mesh()->faces())
		if( minAngle(f) < RADIANS(theta) )
			fclass[f] = CURVE;

	// 2) Assign vertex classification from faces
	classifyVertsFromFaces();

	// 3) Blur classification
	foreach(Vertex v, mesh()->vertices()){
		QSet<Vertex> k_rings;
		collectRing(v, k_rings, k);

		// Get median
		QVector<double> nighbours;
		foreach(Vertex v, k_rings) nighbours.push_back(vclass[v]);
		vclass[v] = median(nighbours);

		if(isVisualize) point_soup->addPoint(points[v], qtJetColorMap(vclass[v]));
	}

	// 4) Cutoff 
	foreach(Vertex v, mesh()->vertices()){
		if(vclass[v] > 0.5){
			Surface_mesh::Face_around_vertex_circulator adjF(mesh(), v), fend = adjF;
			do { fclass[adjF] = SHEET; } while(++adjF != fend);
		}
	}

	// 5) Filter very small components
	if(minRadius > 0)
	{
		GrowRegions();
		foreach(Region r, sheet_regions)
		{
			QBox3D region_bbox;

			foreach(Face f, r)
				region_bbox.unite( center(f) );

			if(region_bbox.size().length() < minRadius)
				invertRegion(r);
		}
	}

	// 6) Find set of faces in all regions
	GrowRegions();

	// 7) Split curve regions into parts
	int curve_count = (int)curve_regions.size();
	for(int i = 0; i < curve_count; i++)
		splitCurveRegion( curve_regions[i] );

	// 8) Save as separate meshes
	if(isExtract){
		int cid = 0, sid = 0;

		document()->pushBusy();
		// Curves
		for(int i = 0; i < (int) curve_regions.size(); i++)
		{
			if(!curve_regions[i].size()) continue;

			SurfaceMeshModel * m = new SurfaceMeshModel("", QString("%1_curve%2").arg(mesh()->name).arg(cid++));
			setMeshFromRegion(curve_regions[i], m);
			m->updateBoundingBox();
			document()->addModel(m);
		}

		// Sheets
		for(int i = 0; i < (int) sheet_regions.size(); i++)
		{
			if(!sheet_regions[i].size()) continue;

			SurfaceMeshModel * m = new SurfaceMeshModel("", QString("%1_sheet%2").arg(mesh()->name).arg(sid++));
			setMeshFromRegion(sheet_regions[i], m);
			m->updateBoundingBox();
			document()->addModel(m);
		}

		document()->popBusy();
	}

	// Debug
	if(isVisualize){
		double e = mesh()->bbox().size().length() * 0.05 * 1;
		QVector3D shift(e,e,e);
		static std::vector< std::vector<double> > rclr = randomColors(curve_regions.size());
		
		// Draw curves
		for(int r = 0; r < (int)curve_regions.size(); r++){
			foreach(Face f, curve_regions[r]){
				QVector<QVector3D> p;
				VertFaceIter vit = mesh()->vertices(f), vend = vit;
				do{ p.push_back(shift + points[vit]); } while(++vit != vend);
				for(int i = 0; i < p.size(); i++)
					lines->addLine(p[i],p[(i+1) % p.size()], QColor::fromRgbF(rclr[r][0], rclr[r][1], rclr[r][2]));
			}
		}
		// Draw sheets
		foreach(Region region, sheet_regions){
			foreach(Face f, region){
				QVector<QVector3D> p;
				VertFaceIter vit = mesh()->vertices(f), vend = vit;
				do{ p.push_back(-shift + points[vit]); } while(++vit != vend);
				poly_soup->addPoly(p);
			}
		}

		drawArea()->addRenderObject(poly_soup);
		drawArea()->addRenderObject(lines);
		drawArea()->addRenderObject(point_soup);
	}

	qDebug() << (QString("Total = %1  |  Sheets found = %2  |  Curves = %3")
		.arg(sheet_regions.size() + curve_regions.size()).arg(sheet_regions.size()).arg(curve_regions.size()));

	// Experiments:
	if(isExtract)
		doGraph();

}

void segment::doGraph()
{
	QMap<QString, int> id;
	Graph<int,int> g;

	// Assign target IDs
	foreach(Face f, mesh()->faces())
	{
		id[ faceTarget[f] ] ++;
	}

	// Add edges
	foreach(Edge e, mesh()->edges())
	{
		Face f1 = mesh()->face(mesh()->halfedge(e,0));
		Face f2 = mesh()->face(mesh()->halfedge(e,1));

		QString target1 = faceTarget[f1];
		QString target2 = faceTarget[f2];

		if(target1 == target2) continue;

		g.AddEdge(id[target1], id[target2], 1);
	}

	std::set< int > nodes = g.GetNodes();
	std::vector< Graph<int,int>::Edge > edges = g.GetEdges();

	QFile out(QString("%1_graph.gexf").arg(mesh()->name));
	out.open(QIODevice::WriteOnly|QIODevice::Text);
	out.write("<?xml version='1.0' encoding='UTF-8'?> <gexf xmlns='http://www.gexf.net/1.2draft' version='1.2'> <graph mode='static' defaultedgetype='undirected'>");

	out.write("<nodes>"); // Start nodes
	foreach(int n, nodes)
	{
		out.write( qPrintable(QString("<node id='%1' label='%2'/>").arg(n).arg(id.key(n).split('_').last())) );
	}
	out.write("</nodes>"); // End nodes

	out.write("<edges>"); // Start edges
	int eid = 0;
	for(std::vector< Graph<int,int>::Edge >::iterator e = edges.begin(); e != edges.end(); e++)
	{
		out.write( qPrintable(QString("<edge id='%1' source='%2' target='%3'/>").arg(eid++).arg(e->index).arg(e->target)) );
	}
	out.write("</edges>"); // End edges

	out.write("</graph></gexf>");
}

void segment::classifyVertsFromFaces()
{
	foreach(Vertex v, mesh()->vertices()){
		Surface_mesh::Face_around_vertex_circulator adjF(mesh(), v), fend = adjF;
		do { if(fclass[adjF] == SHEET) { vclass[v] = 1.0; break; } } while(++adjF != fend);
	}
}

void segment::GrowRegions()
{
	// Clear regions
	curve_regions.clear();
	sheet_regions.clear();

	resetVisitFlag();

	foreach(Face f, mesh()->faces())
	{
		// Skip visited
		if(fvisited[f]) 
			continue;

		int currClass = fclass[f];

		HalfedgeFaceIter h(mesh(), f), eend = h;
		do{ 
			// Check if class of adjacent face doesn't match current face
			if(fclass[ mesh()->face(mesh()->opposite_halfedge(h)) ] != currClass)
			{
				if(currClass == CURVE) curve_regions.push_back( growRegion(f) );
				if(currClass == SHEET) sheet_regions.push_back( growRegion(f) );
				break;
			}
		} while(++h != eend);
	}

	// For single class classification of entire mesh
	if( !curve_regions.size() ){
		Face f(0); 
		int currClass = fclass[f];

		if(currClass == CURVE) curve_regions.push_back( growRegion(f) );
		if(currClass == SHEET) sheet_regions.push_back( growRegion(f) );
	}
}

Region segment::growRegion( Face seedFace )
{
	Region region;

	QStack<Face> toVisit;
	toVisit.push(seedFace);

	int currClass = fclass[seedFace];

	// Recursive grow:
	while( !toVisit.isEmpty() )
	{
		// Get first item, add to growing region
		Face f = toVisit.pop();
		region.insert(f);

		// Check adjacent faces
		HalfedgeFaceIter h(mesh(), f), eend = h;
		do{ 
			Face adj = mesh()->face(mesh()->opposite_halfedge(h));

			// If adjacent is not visited and has same class, push to stack
			if( !fvisited[adj] && fclass[adj] == currClass )
				toVisit.push(adj);
		} while(++h != eend);

		// Mark as visited
		fvisited[f] = true;
	}

	return region;
}

void segment::invertRegion( Region & r )
{
	int currClass = fclass[*r.begin()];
	int toClass = (currClass == SHEET) ? CURVE : SHEET;

	foreach(Face f, r)
		fclass[f] = toClass;
}

void segment::setMeshFromRegion( Region & r, SurfaceMeshModel * m)
{
	// Get set of vertices
	QSet<Vertex> verts; QMap< Face, QVector<Vertex> > adjV;
	foreach(Face f, r){
		VertFaceIter v(mesh(), f), vend = v;
		do{ verts.insert(v); adjV[f].push_back(v); } while (++v != vend);
	}

	// VERTICES: Map them to ordered numbers, and add to new mesh
	QMap<Vertex,Vertex> vmap;
	QMap<Vertex,Vertex> inv_vmap;
	QSetIterator<Vertex> vi(verts);

	while(vi.hasNext()){
		Vertex v = vi.next();
		Vertex newV = Vertex(vmap.size());
		vmap[v] = newV;
		m->add_vertex(points[v]);
		inv_vmap[newV] = v;
	}

	// FACES
	QMap<Face,Face> fmap;
	foreach(Face f, r){
		if(mesh()->valence(f) == 3)
		{
			fmap[Face(fmap.size())] = f;
			m->add_triangle(vmap[adjV[f][0]], vmap[adjV[f][1]], vmap[adjV[f][2]]);
			faceTarget[f] = m->name;
		}
	}

	// Mapping to original surface
	SurfaceMeshModel::Vertex_property<Vertex> surface_vmap = m->vertex_property<Vertex>("v:original", Vertex());
	SurfaceMeshModel::Face_property<Face> surface_fmap = m->face_property<Face>("f:original", Face());

	foreach(Vertex v, m->vertices())
		surface_vmap[v] = inv_vmap[v];
	
	foreach(Face f, m->faces())
		surface_fmap[f] = fmap[f];
}

void segment::collectRing( Vertex v, QSet<Vertex> & set, int level )
{
	if(level > 0){
		VertIter vi(mesh(), v), vend = vi;
		do{	collectRing(vi, set, level - 1); set.insert(vi); } while(++vi != vend);
	}
	else
		set.insert(v);
}

double segment::median( QVector<double> vec )
{
	typedef vector<int>::size_type vec_sz;
	vec_sz size = vec.size();
	if (size == 0) return DBL_MAX;
	qSort(vec.begin(), vec.end());
	vec_sz mid = size / 2;
	return size % 2 == 0 ? (vec[mid] + vec[mid-1]) / 2 : vec[mid];
}

double segment::minAngle(Face f)
{
	double minAngle(DBL_MAX);

	HalfedgeFaceIter h(mesh(), f), eend = h;
	do{ 
		Vector3 a = points[mesh()->to_vertex(h)];
		Vector3 b = points[mesh()->from_vertex(h)];
		Vector3 c = points[mesh()->to_vertex(mesh()->next_halfedge(h))];

		double d = dot((b-a).normalized(), (c-a).normalized());
		double angle = acos(qRanged(-1.0, d, 1.0));

		minAngle = qMin(angle, minAngle);
	} while(++h != eend);

	return minAngle;
}

Vector3 segment::center( Face f )
{
	HalfedgeFaceIter h(mesh(), f), eend = h;

	Vector3 a = points[mesh()->to_vertex(h)];
	Vector3 b = points[mesh()->from_vertex(h)];
	Vector3 c = points[mesh()->to_vertex(mesh()->next_halfedge(h))];

	return (a + b + c) / 3.0;
}

#include "CurveskelHelper.h"
#include "MyPriorityQueue.h"
using namespace CurveskelTypes;

void segment::splitCurveRegion(Region & r)
{
	SurfaceMeshModel m;
	setMeshFromRegion(r, &m);
	SurfaceMeshTypes::Vector3VertexProperty m_points = m.vertex_property<SurfaceMeshTypes::Vector3>(SurfaceMeshTypes::VPOINT);
	SurfaceMeshModel::Face_property<Face> original_face = m.face_property<Face>("f:original", Face());

	// Perform edge collapses until we get 1D curves:
	CurveskelModel skel("");

	// 1) Transfer vertices
	foreach(SurfaceMeshTypes::Vertex v, m.vertices()){
		SurfaceMeshTypes::Point p = m_points[v];
		skel.add_vertex(CurveskelTypes::Point(p[0], p[1], p[2]) );
	}

	// 2) Faces
	foreach(SurfaceMeshTypes::Face f, m.faces()){
		std::vector<CurveskelTypes::Vertex> m_vertices;
        Surface_mesh::Vertex_around_face_circulator vit = m.vertices(f),vend=vit;
		do{ m_vertices.push_back(CurveskelTypes::Vertex( SurfaceMeshTypes::Vertex(vit).idx() )); } while(++vit != vend);
		skel.add_face( m_vertices );
	}

	CurveskelHelper h( &skel );
	CurveskelTypes::ScalarEdgeProperty elen = h.computeEdgeLengths();
	CurveskelModel::Vertex_property< std::set<CurveskelTypes::Vertex> > vrecord = skel.vertex_property< 
			std::set<CurveskelTypes::Vertex> >("v:collapse-from", std::set<CurveskelTypes::Vertex>());
	
	// 3) Add to priority queue
	MyPriorityQueue queue( &skel );
	foreach(CurveskelTypes::Edge edge, skel.edges())
		queue.insert(edge, elen[edge]);

	// first add yourself to the set
	foreach(CurveskelTypes::Vertex v, skel.vertices())
		vrecord[v].insert(v);

	/// 4) Collapse cycle
	while (!queue.empty()){

		/// Retrieve shortest edge
		CurveskelModel::Edge e = queue.pop();

		/// Make sure edge was not already dealt with by previous collapses
		if(!skel.has_faces(e) || skel.is_deleted(e) || !skel.is_valid(e))
			continue;

		CurveskelModel::Vertex v1 = skel.vertex(e, 0); // 'v1' will be deleted
		CurveskelModel::Vertex v2 = skel.vertex(e, 1);

		/// Do collapse
		skel.collapse(e);

		/// record collapsed vertex
		vrecord[v2].insert(v1);

		// carry its records too
		vrecord[v2].insert(vrecord[v1].begin(), vrecord[v1].end());

		/// Update length of edges incident to remaining vertex
		CurveskelModel::Edge_around_vertex eit (&skel, v2);

		while(!eit.end())
		{
			CurveskelModel::Edge edge = eit;

			double newLength = skel.edge_length(edge);

			// If edge still in queue, update its position
			if(queue.has(edge))
				queue.update(edge, newLength);

			++eit;
		}
	}

	// Save mapping between the region's surface and underlying skeleton curve graph
	QMap< int, std::set<int> > vmap;
	int skel_vidx = 0;
	foreach(CurveskelModel::Vertex v, skel.vertices()){
		if(!skel.is_deleted(v)){
			foreach(CurveskelModel::Vertex vj, vrecord[v])
				vmap[ skel_vidx ].insert( vj.idx() );
			skel_vidx++;
		}
	}
	skel.garbage_collection();

	// Prepare variables for visiting branches and assigning IDs
	CurveskelTypes::BoolVertexProperty visited = skel.vertex_property<bool>("v:visited", false);
	CurveskelTypes::IntVertexProperty branchID = skel.vertex_property<int>("v:branch", false);
	std::set<CurveskelTypes::Vertex> junctions = skel.junctions();

	// Mark junctions as visited
	foreach(CurveskelTypes::Vertex v, junctions)
		visited[v] = true;

	int branch_id = 0;

	// Go over separate individual branches and assign IDs
	foreach(CurveskelTypes::Vertex vi, skel.vertices())
	{
		if(visited[vi]) continue;

		QStack< CurveskelTypes::Vertex > stack;
		stack.push(vi);

		while(!stack.empty()){
			CurveskelTypes::Vertex cur = stack.pop();

			foreach(CurveskelTypes::Vertex vj, skel.adjacent_set(cur)){
				branchID[vj] = branch_id;

				if( !visited[vj] ){
					stack.push(vj);
					visited[vj] = true;
				}
			}
		}

		branch_id++;
	}

	// Propagate branch ID to original structure
	RegionVector sub_regions;
	sub_regions.resize( branch_id );

	SurfaceMeshModel::Face_property<bool> face_assigned = m.face_property<bool>("f:assignedID", false);

	foreach(CurveskelModel::Vertex skel_v, skel.vertices()){
		int curr_id = branchID[ skel_v ];

		// For each surface vertex belonging to skeleton vertex 'skel_v'
		foreach(int vi, vmap[ skel_v.idx() ])
		{
			// For each adjacent face of vi
			Surface_mesh::Face_around_vertex_circulator adjF( &m, Surface_mesh::Vertex(vi) ), fend = adjF;
			do {
				if(!face_assigned[ adjF ])
				{
					sub_regions[ curr_id ].insert( original_face[adjF] ); 
					face_assigned[ adjF ] = true;
				}
			} while(++adjF != fend);
		}
	}

	// Empty out the original region
	r.clear();

	// Add its sub-regions
	foreach(Region r, sub_regions)
		curve_regions.push_back(r);


	// DEBUG
	/*static std::vector< std::vector<double> > rc = randomColors(branch_id + 1);
	foreach(CurveskelTypes::Vertex v, skel.vertices()){
		int i = branchID[v];

		CurveskelTypes::Vector3 pnt = skel.get_vertex_property<CurveskelTypes::Vector3>(CurveskelTypes::VPOINT)[v];
		QVector3D p (pnt.x(), pnt.y(), pnt.z());
		
		QColor color = QColor::fromRgbF(rc[i][0],rc[i][1],rc[i][2]);
		drawArea()->drawPoint(p,10, color);
	}*/

	// DEUBG:
	/*CurveskelModel * s = new CurveskelModel;
	foreach(CurveskelModel::Vertex v, skel.vertices()) s->add_vertex(skel.get_vertex_property<CurveskelTypes::Vector3>(CurveskelTypes::VPOINT)[v]);
	foreach(CurveskelModel::Edge e, skel.edges()) s->add_edge(skel.vertex(e, 0), skel.vertex(e, 1));

	foreach(CurveskelTypes::Vertex v, s->junctions())
	{
		CurveskelTypes::Vector3 pnt = skel.get_vertex_property<CurveskelTypes::Vector3>(CurveskelTypes::VPOINT)[v];
		QVector3D p (pnt.x(), pnt.y(), pnt.z());
		drawArea()->drawPoint(p,10);
	}

	document()->addModel(s);*/
}


Q_EXPORT_PLUGIN(segment)
