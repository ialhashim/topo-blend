#include "segment.h"
#include "StarlabDrawArea.h"
#include "CustomDrawObjects.h"
#include <QStack>

#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )
#define RADIANS(deg)    ((deg)/180.0 * M_PI)
#define DEGREES(rad)    ((rad)/M_PI * 180.0)

typedef Surface_mesh::Halfedge_around_face_circulator HalfedgeFaceIter;
typedef Surface_mesh::Vertex_around_face_circulator VertFaceIter;
typedef Surface_mesh::Face_around_vertex_circulator FaceVertIter;
typedef Surface_mesh::Vertex_around_vertex_circulator VertIter;

uint qHash( const Face &key ){return qHash(key.idx()); }
uint qHash( const Vertex &key ){return qHash(key.idx()); }

void segment::initParameters(RichParameterSet* pars){
	pars->addParam( new RichFloat("angle_threshold", 10.0f, "Angle threshold"));
	pars->addParam( new RichInt("k_nighbours", 3, "K-levels"));
	pars->addParam( new RichBool("visualize", false, "Visualize regions"));
	pars->addParam( new RichBool("extract", true, "Extract regions"));

	// Create helper object to access points and edge lengths
	SurfaceMeshHelper h(mesh());
	points = h.getVector3VertexProperty(VPOINT);
	farea = h.computeFaceAreas();
	elen = h.computeEdgeLengths();
}

void segment::initMesh()
{
	resetClassfication();
	resetVisitFlag();
	resetVertexClass();
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

void segment::applyFilter(RichParameterSet* pars)
{
	// Get user specified parameters
	double theta = pars->getFloat("angle_threshold");
	int k = pars->getInt("k_nighbours");
	bool visualize = pars->getBool("visualize");
	bool extract = pars->getBool("extract");

	// Perform segmentation
	performCurveSheetSegmentation(theta, k, extract, visualize);
}

void segment::performCurveSheetSegmentation(double theta, int k, bool isExtract, bool isVisualize)
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

		if(isVisualize) point_soup->addPoint(points[v], qtColdColor(vclass[v]));
	}

	// 4) Cutoff 
	foreach(Vertex v, mesh()->vertices()){
		if(vclass[v] > 0.5){
			Surface_mesh::Face_around_vertex_circulator adjF(mesh(), v), fend = adjF;
			do { fclass[adjF] = SHEET; } while(++adjF != fend);
		}
	}

	// 5) Find set of faces in all regions
	GrowRegions();

	// 6) Save as separate meshes
	if(isExtract){
		document()->pushBusy();
		// Curves
		for(int i = 0; i < (int) curve_regions.size(); i++)
		{
			SurfaceMeshModel * m = new SurfaceMeshModel("", QString("%1_curve_%2").arg(mesh()->name).arg(i));
			setMeshFromRegion(curve_regions[i], m);
            m->updateBoundingBox();
			document()->addModel(m);
		}

		// Sheets
		for(int i = 0; i < (int) sheet_regions.size(); i++)
		{
			SurfaceMeshModel * m = new SurfaceMeshModel("", QString("%1_sheet_%2").arg(mesh()->name).arg(i));
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
		// Draw curves
		foreach(Region region, curve_regions){
			foreach(Face f, region){
				QVector<QVector3D> p;
				VertFaceIter vit = mesh()->vertices(f), vend = vit;
				do{ p.push_back(shift + points[vit]); } while(++vit != vend);
				for(int i = 0; i < p.size(); i++){
					lines->addLine(p[i],p[(i+1) % p.size()]);
				}
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
		Face f(0); int currClass = fclass[f];
		resetVisitFlag();
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
	QSetIterator<Vertex> vi(verts);
	while(vi.hasNext()){
		Vertex v = vi.next();
		vmap[v] = Vertex(vmap.size());
		m->add_vertex(points[v]);
	}

	// FACES
	foreach(Face f, r){
		if(mesh()->valence(f) == 3) 
			m->add_triangle(vmap[adjV[f][0]], vmap[adjV[f][1]], vmap[adjV[f][2]]);
	}
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

Q_EXPORT_PLUGIN(segment)
