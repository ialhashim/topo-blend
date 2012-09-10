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

uint qHash( const Face &key ){return qHash(key.idx()); }

void segment::initParameters(RichParameterSet* pars){
	pars->addParam( new RichFloat("angle_threshold", 0.05f, "Angle threshold"));
	pars->addParam( new RichInt("grow_itr", 0, "Grow iterations"));
	pars->addParam( new RichFloat("curve_region_factor", 1.0, "Curve region threshold"));

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
}

void segment::resetClassfication()
{
	mesh()->remove_face_property<FaceClass>(fclass);
	fclass = mesh()->add_face_property<FaceClass>("f:class", SHEET);
}

void segment::resetVisitFlag()
{
	mesh()->remove_face_property<bool>(fvisited);
	fvisited = mesh()->add_face_property("f:visited", false);
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
	// Clear debug artifacts
	drawArea()->deleteAllRenderObjects();

	// Retrieve parameters
	double theta = pars->getFloat("angle_threshold");

	initMesh();
	
	// 1) Check valid faces by minimum angle
	foreach(Face f, mesh()->faces())
		if( minAngle(f) < RADIANS(theta) )
			fclass[f] = CURVE;

	// 2) Filter bad faces marked as curves
	/*for(int i = 0; i < pars->getInt("grow_itr"); i++)
	{
		QVector<Vertex> toSheet;

		// Find vertices to convert
		foreach(Vertex v, mesh()->vertices()){
			int sheet_count = 0;
			
			Surface_mesh::Face_around_vertex_circulator adjF(mesh(), v), fend = adjF;
			do { if( fclass[adjF] == SHEET) sheet_count++ ; } while(++adjF != fend);

			if(sheet_count < mesh()->valence(v) && sheet_count > mesh()->valence(v) * 0.5)
				toSheet.push_back(v);
		}
		
		// Convert all adjacent faces of 'v' to sheet
		foreach(Vertex v, toSheet){
			Surface_mesh::Face_around_vertex_circulator adjF(mesh(), v), fend = adjF;
			do { fclass[adjF] = SHEET; } while(++adjF != fend);
		}
	}*/

	// 3) Classify by growing regions
	GrowRegions();
	
	// 4) Fix curve region misclassification by examining region boundry
	if( curve_regions.size() )
	{
		// Compute perimeter of largest region
		/*int maxRegionSize = -1;
		int maxIdx = 0;

		for(int i = 0; i < (int) curve_regions.size(); i++){
			if(curve_regions[i].size() > maxRegionSize){
				maxRegionSize = curve_regions[i].size();
				maxIdx = i;
			}
		}

		double maxPerimeter = regionPerimeter(curve_regions[maxIdx]);
		double maxArea = regionArea(curve_regions[maxIdx]);

		// Reclassify based on threshold
		double factor = pars->getFloat("curve_region_factor");

		foreach(Region r, curve_regions){
			if(regionPerimeter(r) > maxPerimeter * factor){
				QSetIterator<Face> i(r);
				while (i.hasNext()){
					fclass[i.next()] = SHEET;
				}
			}
		}

		// Final pass
		if(factor > 0){
			resetVisitFlag();
			GrowRegions();
		}*/

		foreach(Region r, curve_regions){

		}
	}
	else
	{
		// If we only single classification, assign as single region (of face[0] class)
		Face f(0); FaceClass currClass = fclass[f];
		resetVisitFlag();
		if(currClass == CURVE) curve_regions.push_back( growRegion(f) );
		if(currClass == SHEET) sheet_regions.push_back( growRegion(f) );
	}

	// 3) Debug regions
	PolygonSoup * poly_soup = new PolygonSoup;
	LineSegments * lines = new LineSegments;
	double e = mesh()->bbox().size().length() * 0.05 * 1;
	QVector3D shift(e,e,e);
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

		FaceClass currClass = fclass[f];

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
}

Region segment::growRegion( Face seedFace )
{
	Region region;

	QStack<Face> toVisit;
	toVisit.push(seedFace);

	FaceClass currClass = fclass[seedFace];

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

double segment::regionPerimeter(Region & region)
{
	double length = 0;

	// Find class of region
	QSetIterator<Face> i(region);
	FaceClass currClass = fclass[i.peekNext()];

	while (i.hasNext()){
		Face f = i.next();

		HalfedgeFaceIter h(mesh(), f), eend = h;
		do{ 
			Face adj = mesh()->face(mesh()->opposite_halfedge(h));

			// If adjacent is not same class, add edge length to sum
			if( fclass[adj] != currClass )
				length += elen[mesh()->edge(h)];
		} while(++h != eend);
	}

	return length;
}

double segment::regionArea(Region & region)
{
	double area = 0;

	foreach(Face f, region)
		area += farea[f];

	return area;
}

int segment::regionBoundries(Region & region)
{
	int numBoundries = 0;



	return numBoundries;
}

Q_EXPORT_PLUGIN(segment)
