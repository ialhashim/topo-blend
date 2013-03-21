#include <QFileDialog>
#include <QStack>

#include "nurbs_plugin_global.h"

#include "nurbs_plugin.h"
#include "NanoKdTree.h"
#include "StructureGraph.h"

#include "interfaces/ModePluginDockWidget.h"

#include "BoundaryFitting.h"

QVector<QColor> randColors;

//#include "LSCM.h"
//#include "LinABF.h"

SurfaceMesh::SurfaceMeshModel * entireMesh;
Vector3VertexProperty entirePoints;

QString groupID = "";
SurfaceMesh::SurfaceMeshModel * m = NULL;
RichParameterSet * mcf_params = NULL;

#include "../CustomDrawObjects.h"
PolygonSoup ps;
SphereSoup spheres;
VectorSoup vs;

#include "PCA.h"

#include "StructureGraph.h"

#define RADIANS(deg)    ((deg)/180.0 * M_PI)

void nurbs_plugin::create()
{
	if(widget) return;

	entireMesh = (SurfaceMeshModel*)document()->selectedModel();
	entirePoints = entireMesh->vertex_property<Vector3>("v:point");

    ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
    widget = new NURBSTools(this);
    dockwidget->setWidget(widget);
    dockwidget->setWindowTitle(widget->windowTitle());
    mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);

	for(int i = 0; i < 10; i++)
		randColors.push_back(qRandomColor());

	loadGroupsFromOBJ();

	m = mesh();
	points = m->vertex_property<Vector3>("v:point");

	graph = new Structure::Graph;

	//buildSamples();
}

void nurbs_plugin::decorate()
{
	// Draw OBB
	//mesh_obb.draw();

	for(int i = 0; i < (int)curves.size(); i++)
	{
		NURBS::CurveDraw::draw( &curves[i], randColors[i%randColors.size()], true );
	}

	for(int i = 0; i < (int)rects.size(); i++)
	{
		NURBS::SurfaceDraw::draw( &rects[i], randColors[i%randColors.size()], true );
	}

	if(graph) graph->draw();

	ps.draw();
	vs.draw();

	spheres.draw();
}

void nurbs_plugin::doFitCurve()
{
    qDebug() << "Curve fitting..";

	points = m->vertex_property<Vector3>("v:point");
	mesh_obb = OBB_Volume( m );
	std::vector<Vec3d> corners = mesh_obb.corners();
	Vec3d diag = mesh_obb.extents();
	Vec3d from = mesh_obb.center() + diag;
	Vec3d to = mesh_obb.center() - diag;
    NURBS::NURBSCurved c = NURBS::NURBSCurved::createCurve( from, to, widget->uCount() );
	std::vector<Vec3d> mesh_points;
	foreach(Vertex v, m->vertices()) mesh_points.push_back( points[v] );
	basicCurveFit(c, mesh_points);
	c.mCtrlPoint = smoothPolyline( c.mCtrlPoint, 2 );

	curves.push_back( c );

	drawArea()->updateGL();
}

void nurbs_plugin::basicCurveFit( NURBS::NURBSCurved & curve, std::vector<Vec3d> pnts )
{
	int high = curve.mCtrlPoint.size() - 1;
	int low = 0;

	Vec3d planeNormal = (curve.mCtrlPoint[high] - curve.mCtrlPoint[low]).normalized();
	QMap<double, Vec3d> distsLow, distsHigh;
	for(int i = low; i <= high; i++){
		for(int j = 0; j < (int)pnts.size(); j++){
			distsLow[abs(dot( planeNormal, (pnts[j] - curve.mCtrlPoint[low]) ))] = pnts[j];
			distsHigh[abs(dot( planeNormal, (pnts[j] - curve.mCtrlPoint[high]) ))] = pnts[j];
		}
	}
	curve.mCtrlPoint[low] = distsLow.values().front();
	curve.mCtrlPoint[high] = distsHigh.values().front();

	basicCurveFitRecursive(curve,pnts,high,low);
}

void nurbs_plugin::basicCurveFitRecursive( NURBS::NURBSCurved & curve, std::vector<Vec3d> pnts, int high, int low )
{
	int mid = ((high - low) * 0.5) + low;

	if(high <= low || mid == low || mid == high) return;

	qDebug() << QString("High = %1  Mid = %2  Low = %3").arg(high).arg(mid).arg(low);

	Vec3d planeCenter = (curve.mCtrlPoint[high] + curve.mCtrlPoint[low]) * 0.5;
	Vec3d planeNormal = (curve.mCtrlPoint[high] - curve.mCtrlPoint[low]).normalized();

	QMap<double, Vec3d> dists;
	for(int i = low; i <= high; i++)
	{
		for(int j = 0; j < (int)pnts.size(); j++)
		{
			double d = abs(dot( planeNormal, (pnts[j] - planeCenter) ));
			dists[d] = pnts[j];
		}
	}

	Vec3d closestPoint = dists.values().front();
	curve.mCtrlPoint[mid] = closestPoint;

	Vec3d deltaLow = (curve.mCtrlPoint[mid] - curve.mCtrlPoint[low]) / (mid - (low + 1));
	for(int i = low + 1; i < mid; i++)
	{
		curve.mCtrlPoint[i] = curve.mCtrlPoint[i-1] + deltaLow;
	}

	Vec3d deltaHigh = (curve.mCtrlPoint[high] - curve.mCtrlPoint[mid]) / (high - (mid + 1));
	for(int i = mid + 1; i < high; i++)
	{
		curve.mCtrlPoint[i] = curve.mCtrlPoint[i-1] + deltaHigh;
	}

	basicCurveFitRecursive(curve, pnts, mid, low);
	basicCurveFitRecursive(curve, pnts, high, mid);
}

void nurbs_plugin::doFitSurface()
{
	this->rects.clear();

	points = m->vertex_property<Vector3>("v:point");

	// Pick a side by clustering normals

	// 1) Find edge with flat dihedral angle
	double angleThreshold = deg_to_rad( 10.0 );

	Halfedge startEdge = m->halfedges_begin();
	foreach(Halfedge h, m->halfedges()){
		if(calc_dihedral_angle( m, h ) < angleThreshold){
			startEdge = h;
			break;
		}
	}

	// 2) Grow region by comparing difference of adjacent dihedral angles
	SurfaceMesh::Model::Vertex_property<bool> vvisited = m->add_vertex_property<bool>("v:visited", false);

	QStack<SurfaceMesh::Model::Vertex> to_visit;
	to_visit.push( m->to_vertex(startEdge) );

	while(!to_visit.empty())
	{
		Vertex cur_v = to_visit.pop();
		if( vvisited[cur_v] ) continue;
		vvisited[cur_v] = true;

		// Sum of angles around
		double sumAngles = 0.0;
		foreach(Halfedge hj, m->onering_hedges(cur_v)){
			sumAngles += abs(calc_dihedral_angle( m, hj ));
		}

		foreach(Halfedge hj, m->onering_hedges(cur_v))
		{
			Vertex vj = m->to_vertex(hj);
			if(sumAngles < angleThreshold)
				to_visit.push(vj);
			else
				vvisited[vj];
		}
	}


	// Get filtered inner vertices of side
	std::set<Vertex> inner;
	std::set<Vertex> border;
	int shrink_count = 1;
	for(int i = 0; i < shrink_count; i++)
	{
		std::set<Vertex> all_points;
		foreach(Vertex v, m->vertices())
			if(vvisited[v]) all_points.insert(v);

		border.clear();
		foreach(Vertex v, m->vertices()){
			if(vvisited[v]){
				foreach(Halfedge hj, m->onering_hedges(v)){
					Vertex vj = m->to_vertex(hj);
					if(!vvisited[vj])
						border.insert(vj);
				}
			}
		}

		inner.clear();
		std::set_difference(all_points.begin(), all_points.end(), border.begin(), border.end(),
			std::inserter(inner, inner.end()));

		// Shrink one level
		foreach(Vertex vv, border){
			foreach(Halfedge hj, m->onering_hedges(vv))
				vvisited[ m->to_vertex(hj) ] = false;
		}
	}

	SurfaceMesh::Model * submesh = NULL;
	
	bool isOpen = false;
	foreach(Vertex v, m->vertices()){
		if(m->is_boundary(v)){
			isOpen = true;
			break;
		}
	}

	if(!isOpen)
	{
		// Collect inner faces
		std::set<Face> innerFaces;
		std::set<Vertex> inFacesVerts;
		foreach(Vertex v, inner)
		{
			foreach(Halfedge hj, m->onering_hedges(v)){
				Face f = m->face(hj);
				innerFaces.insert(f);
				Surface_mesh::Vertex_around_face_circulator vit = m->vertices(f),vend=vit;
				do{ inFacesVerts.insert( Vertex(vit) ); } while(++vit != vend);
			}
		}

		// Create sub-mesh
		submesh = new SurfaceMesh::Model("SideFlat.obj","SideFlat");
	
		// Add vertices
		std::map<Vertex,Vertex> vmap;
		foreach(Vertex v, inFacesVerts){
			vmap[ v ] = Vertex(vmap.size());
			submesh->add_vertex( points[v] );
		}

		// Add faces
		foreach(Face f, innerFaces){
			std::vector<Vertex> verts;
			Surface_mesh::Vertex_around_face_circulator vit = m->vertices(f),vend=vit;
			do{ verts.push_back( Vertex(vit) ); } while(++vit != vend);
			submesh->add_triangle( vmap[verts[0]], vmap[verts[1]], vmap[verts[2]] );
		}
	}
	else
	{
		submesh = m;
	}

	Vector3VertexProperty pos_submesh = submesh->vertex_property<Vector3>(VPOINT);

	if(m != submesh)
		document()->addModel(submesh);

	// Find 4 corners of a NURBS rect on mesh boundary
	submesh->updateBoundingBox();
	double resolution = submesh->bbox().size().length() * widget->resolution();
	BoundaryFitting bf( submesh, resolution );

	//PointSoup * ps = new PointSoup;
	//foreach(Vertex v, submesh->vertices())
	//	ps->addPoint( pos_submesh[v], qtJetColorMap(1.0 - bf.dists[v]) );
	//drawArea()->addRenderObject(ps);

	Array2D_Vector3 cp = bf.lines;
	Array2D_Real cw(cp.size(), Array1D_Real(cp.front().size(), 1.0));
	int degree = 3;

	rects.push_back( NURBS::NURBSRectangled(cp,cw,degree,degree,false,false,true,true) );

	drawArea()->updateGL();
}

void nurbs_plugin::clearAll()
{
	curves.clear();
	rects.clear();
}

void nurbs_plugin::saveAll()
{
	//Structure::Graph g;
	//foreach(NURBS::NURBSCurved curve, curves)
	//	g.addNode( new Structure::Curve(curve, m->name) );
	//foreach(NURBS::NURBSRectangled rect, rects)
	//	g.addNode( new Structure::Sheet(rect, m->name) );
	//QString filename = QFileDialog::getSaveFileName(0, tr("Save Model"), 
	//	mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));
	//g.saveToFile(filename);

	foreach(Structure::Node * n, graph->nodes)
	{
		QString nodeID = n->id;
		SurfaceMeshModel * nodeMesh = extractMesh( nodeID );

		// Assign sub-mesh to node
		n->property["mesh_filename"] = entireMesh->name + "/" + nodeID + ".obj";
		n->property["mesh"].setValue( nodeMesh );
	}

	QString folderPath = QFileDialog::getExistingDirectory();
	QDir::setCurrent( folderPath );
	
	QString filename = folderPath + "/" + entireMesh->name + ".xml";
	graph->saveToFile( filename );
}

void nurbs_plugin::doFitSurface_old()
{
	points = m->vertex_property<Vector3>("v:point");

    qDebug() << "Surface fitting..";

	mesh_obb = OBB_Volume( m );

	Vec3d center = mesh_obb.center();
	std::vector<Vec3d> axisNormalized = mesh_obb.axis();
	Vec3d sides = 2.0 * mesh_obb.extents();
	QVector< QPair< double, Vec3d > > axis;
	for(int i = 0; i < 3; i++)
		axis.push_back( qMakePair(sides[i], axisNormalized[i]) );
	std::sort(axis.begin(),axis.end());
	double width = axis[1].first;
	double length = axis[2].first;
	Vec3d dU = axis[1].second;
	Vec3d dV = axis[2].second;

	// Create coarse B-spline sheet
    NURBS::NURBSRectangled r = NURBS::NURBSRectangled::createSheet( width, length, center, dU, dV, widget->uCount(), widget->vCount() );
	//basicSurfaceFit_old(r);

	rects.push_back(r);
	drawArea()->updateGL();
}

void nurbs_plugin::basicSurfaceFit_old( NURBS::NURBSRectangled & surface, std::vector<Vec3d> pnts )
{
	// Try to minimize distance between control points and point cloud [buggy code..]
	for(int v = 0; v < surface.mNumVCtrlPoints; v++){
		for(int u = 0; u < surface.mNumUCtrlPoints; u++){
			QMap<double, Vec3d> dists;
			/*for(int j = 0; j < (int)pnts.size(); j++)
			{
			double d = abs(dot( planeNormal, (pnts[j] - planeCenter) ));
			*/
			Vec3d & cp = surface.mCtrlPoint[u][v];
			for(int j = 0; j < (int)pnts.size(); j++){
				Vec3d p,t0,t1,normal;
				surface.GetFrame(u,v,p,t0,t1,normal);
				double d = abs(dot( normal, (pnts[j] - p) ));
				dists[d] = pnts[j];
			}
			Vec3d closestPoint = dists.values().front();
			cp = closestPoint;
		}
	}
}

bool nurbs_plugin::keyPressEvent( QKeyEvent* event )
{
	bool used = false;

	if(event->key() == Qt::Key_E)
	{
		NURBS::NURBSCurved & c = curves.back();

		NURBS::NURBSCurved newCurve = NURBS::NURBSCurved::createCurveFromPoints( c.simpleRefine(1) );

		if(curves.size() == 1)
			curves.push_back( newCurve );
		else
			c = newCurve;

		used = true;
	}

	if(event->key() == Qt::Key_R)
	{
		NURBS::NURBSCurved & c = curves.back();

		c = NURBS::NURBSCurved::createCurveFromPoints( c.removeKnots(2) ) ;

		used = true;
	}

	if(event->key() == Qt::Key_Q)
	{
		NURBS::NURBSRectangled & r = rects.back();

		r = NURBS::NURBSRectangled::createSheetFromPoints( r.midPointRefined() );

		used = true;
	}

	if(event->key() == Qt::Key_W)
	{
		//QElapsedTimer timer; timer.start();

		//LinABF linabf(m);
		//mainWindow()->setStatusBarMessage(QString("LinABF time = %1 ms").arg(timer.elapsed()),512);

		//linabf.applyUVTom;

		//used = true;
	}

	if(event->key() == Qt::Key_Z)
	{
		NURBS::NURBSRectangled & r = rects.back();

		r = NURBS::NURBSRectangled::createSheetFromPoints( r.simpleRefine(1,0) );

		used = true;
	}

	if(event->key() == Qt::Key_X)
	{
		NURBS::NURBSRectangled & r = rects.back();

		r = NURBS::NURBSRectangled::createSheetFromPoints( r.simpleRefine(1,1) );

		used = true;
	}

	if(event->key() == Qt::Key_C)
	{
		NURBS::NURBSRectangled & r = rects.back();

		r = NURBS::NURBSRectangled::createSheetFromPoints( r.simpleRemove(r.mNumUCtrlPoints * 0.5,0) );

		used = true;
	}

	if(event->key() == Qt::Key_V)
	{
		NURBS::NURBSRectangled & r = rects.back();

		r = NURBS::NURBSRectangled::createSheetFromPoints( r.simpleRemove(r.mNumVCtrlPoints * 0.5,1) );

		used = true;
	}

	if(event->key() == Qt::Key_Space)
	{
		buildSamples();
		used = true;
	}

	drawArea()->updateGL();

	return used;
}

void nurbs_plugin::buildSamples()
{

	// Curve example
	int degree = 3;
	std::vector<Vec3d> cps;
	int steps = 10;
	double theta = M_PI * 5 / steps;
	double r = M_PI;

	for(int i = 0; i <= steps; i++){
		double x = (double(i) / steps) * r;
		double y = sin(i * theta) * r * 0.25;

		cps.push_back(Vec3d(x - r * 0.5, y, cos(i*theta)));
	}

	std::vector<Scalar> weight(cps.size(), 1.0);

	curves.push_back(NURBS::NURBSCurved(cps, weight, degree, false, true));

	// Rectangular surface
	double w = 1;
	double l = 2;

	int width = w * 5;
	int length = l * 5;

	std::vector< std::vector<Vec3d> > cpts( width, std::vector<Vec3d>(length, 1.0) );
	std::vector< std::vector<Scalar> > weights( width, std::vector<Scalar>(length, 1.0) );

	double omega = M_PI * 3 / qMin(width, length);

	Vec3d surface_pos(0,1,0);

	for(int i = 0; i < width; i++)
		for(int j = 0; j < length; j++){
			double x = double(i) / width;
			double y = double(j) / length;

			double delta = sqrt(x*x + y*y) * 0.5;

			cpts[i][j] = Vec3d(surface_pos.x() + x * w, surface_pos.y() + y * l, delta + sin(j * omega) * qMin(width, length) * 0.02);
		}

	rects.push_back( NURBS::NURBSRectangled(cpts, weights, degree, degree, false, false, true, true) );
}

void nurbs_plugin::prepareSkeletonize()
{
	// Select model to skeletonize
	document()->setSelectedModel( m );

	QMap<QString,FilterPlugin*> plugins = pluginManager()->filterPlugins;

	// Remesh
	QString remeshPlugin = "Isotropic Remesher";
	RichParameterSet * remesh_params = new RichParameterSet;
	plugins[remeshPlugin]->initParameters( remesh_params );
	plugins[remeshPlugin]->applyFilter( remesh_params );
	remesh_params->destructor();

	// Compute MAT
	QString matPlugin = "Voronoi based MAT";
	RichParameterSet * mat_params = new RichParameterSet;
	plugins[matPlugin]->initParameters( mat_params );
	plugins[matPlugin]->applyFilter( mat_params );
	mat_params->destructor();
}

void nurbs_plugin::skeletonizeMesh()
{
	// Pre-process

	//SurfaceMeshModel* m = new SurfaceMeshModel(m->path,"original");
	//m->read( m->path.toStdString() );

	prepareSkeletonize();

	// Contract using MCF skeletonization
	for(int i = 0; i < widget->contractIterations(); i++)
	{
		stepSkeletonizeMesh();
	}

	// Post-process
	{
		// Clean up

		//document()->setSelectedModel( prevModel );
		drawArea()->setRenderer(m,"Flat Wire");
		drawArea()->updateGL();
	}
}

void nurbs_plugin::stepSkeletonizeMesh()
{	
	QString mcfPlugin = "MCF Skeletonization";

	if(!mcf_params){
		mcf_params = new RichParameterSet;
		pluginManager()->filterPlugins[mcfPlugin]->initParameters( mcf_params );

		//mcf_params->setValue("omega_P_0", 0.3f);
	}
	pluginManager()->filterPlugins[mcfPlugin]->applyFilter( mcf_params );

	//drawArea()->setRenderer(m,"Flat Wire");
	//drawArea()->updateGL();
}

void nurbs_plugin::drawWithNames()
{
	// Faces
	foreach( const Face f, entireMesh->faces() )
	{
		// Collect points
		QVector<Vector3> pnts; 
		Surface_mesh::Vertex_around_face_circulator vit = entireMesh->vertices(f),vend=vit;
		do{ pnts.push_back(entirePoints[vit]); } while(++vit != vend);

		glPushName(f.idx());
		glBegin(GL_TRIANGLES);
		foreach(Vector3 p, pnts) glVector3(p);
		glEnd();
		glPopName();
	}
}

void nurbs_plugin::endSelection( const QPoint& p )
{
	drawArea()->defaultEndSelection(p);
}

SurfaceMeshModel * nurbs_plugin::extractMesh( QString gid )
{
	SurfaceMeshModel * subMesh = NULL;

	QVector<int> part = groupFaces[gid];

	// Create copy of sub-part
	subMesh = new SurfaceMeshModel(groupID + ".obj", groupID);
	QSet<int> vertSet;
	QMap<Vertex,Vertex> vmap;
	foreach(int fidx, part){
		Surface_mesh::Vertex_around_face_circulator vit = entireMesh->vertices(Face(fidx)),vend=vit;
		do{ vertSet.insert(Vertex(vit).idx()); } while(++vit != vend);
	}
	foreach(int vidx, vertSet){
		vmap[Vertex(vidx)] = Vertex(vmap.size());
		subMesh->add_vertex( entirePoints[Vertex(vidx)] );
	}
	foreach(int fidx, part){
		std::vector<Vertex> pnts; 
		Surface_mesh::Vertex_around_face_circulator vit = entireMesh->vertices(Face(fidx)),vend=vit;
		do{ pnts.push_back(vmap[vit]); } while(++vit != vend);
		subMesh->add_face(pnts);
	}
	subMesh->updateBoundingBox();
	subMesh->isVisible = false;

	return subMesh;
}

void nurbs_plugin::postSelection( const QPoint& point )
{
	Q_UNUSED(point);
	int selectedID = drawArea()->selectedName();
	if (selectedID == -1){
		ps.clear();
		document()->setSelectedModel(entireMesh);
		return;
	}
	qDebug() << "Selected ID is " << selectedID;

	Vertex selectedVertex = entireMesh->vertices( Surface_mesh::Face(selectedID) );

	QString gid = faceGroup[entireMesh->face(entireMesh->halfedge(selectedVertex)).idx()];

	groupID = gid;
	m = extractMesh( groupID );
	QVector<int> part = groupFaces[gid];

	// Draw selection
	ps.clear();
	foreach(int fidx, part)
	{
		// Collect points
		QVector<QVector3D> pnts; 
		Surface_mesh::Vertex_around_face_circulator vit = entireMesh->vertices(Face(fidx)),vend=vit;
		do{ pnts.push_back(entirePoints[vit]); } while(++vit != vend);

		ps.addPoly(pnts, QColor(255,0,0,100));
	}

}

void nurbs_plugin::loadGroupsFromOBJ()
{
	// Read obj file
	QFile file(mesh()->path);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QTextStream inF(&file);
	int fidx = 0;
	while( !inF.atEnd() ){
		QString line = inF.readLine();
		if(!line.size()) continue;
		if(line.at(0).toAscii() == 'g'){
			QStringList groupLine = line.split(" ");
			QString gid = QString::number(groupFaces.size());
			if(groupLine.size()) gid = groupLine.at(1);
			while(true){
				QString line = inF.readLine();
				if(!line.startsWith("f")) break;
				groupFaces[gid].push_back(fidx);
				faceGroup[fidx] = gid;
				fidx++;
			}
		}
	}
}

Vec3d nurbs_plugin::pole( Vec3d center, double radius, SurfaceMeshModel * part )
{
	Vector3VertexProperty partPoints = part->vertex_property<Vector3>("v:point");

	std::vector<Vec3d> samples;
	QVector<Face> ifaces;
	Vec3d p;

	// intersect sphere with part, collect faces
	foreach( Face f, part->faces() ){
		QVector<Vec3d> pnts; 
		Surface_mesh::Vertex_around_face_circulator vit = part->vertices(Face(f)),vend=vit;
		do{ pnts.push_back(partPoints[vit]); } while(++vit != vend);

		if(TestSphereTriangle(center, radius, pnts[0], pnts[1], pnts[2], p)){
			ifaces.push_back(f);
			foreach(Vec3d s, sampleEdgeTri(pnts[0], pnts[1], pnts[2]))
				samples.push_back(s);
		}
	}

	Vec3d a,b,c;
	return PCA::mainAxis(samples,a,b,c).normalized();
}

std::vector<Vec3d> nurbs_plugin::sampleEdgeTri( Vec3d a, Vec3d b, Vec3d c )
{
	std::vector<Vec3d> samples;
	samples.push_back(a);samples.push_back(b);samples.push_back(c);
	samples.push_back((a+b) * 0.5);
	samples.push_back((b+c) * 0.5);
	samples.push_back((c+a) * 0.5);
	return samples;
}

double nurbs_plugin::minAngle(Face f, SurfaceMeshModel * ofMesh)
{
	double minAngle(DBL_MAX);
	Vector3VertexProperty pts = ofMesh->vertex_property<Vector3>("v:point");

	SurfaceMesh::Model::Halfedge_around_face_circulator h(ofMesh, f), eend = h;
	do{ 
		Vector3 a = pts[ofMesh->to_vertex(h)];
		Vector3 b = pts[ofMesh->from_vertex(h)];
		Vector3 c = pts[ofMesh->to_vertex(ofMesh->next_halfedge(h))];

		double d = dot((b-a).normalized(), (c-a).normalized());
		double angle = acos(qRanged(-1.0, d, 1.0));

		minAngle = qMin(angle, minAngle);
	} while(++h != eend);

	return minAngle;
}

void nurbs_plugin::convertToCurve()
{
	document()->addModel( m );
	document()->setSelectedModel( m );

	prepareSkeletonize(); 

	double theta = 15.0; // degrees

	for(int i = 0; i < 40; i++){
		stepSkeletonizeMesh();
	
		// Decide weather or not to keep contracting based on angle of faces
		bool isDone = true;
		foreach( Face f, m->faces() ){
			if( minAngle(f, m) > RADIANS(theta) )
				isDone = false;
		}
		if(isDone) break;
	}

	drawArea()->deleteAllRenderObjects();

	//mesh_obb = OBB_Volume( m );
	//std::vector<Vec3d> corners = mesh_obb.corners();
	//Vec3d diag = mesh_obb.extents(); 
	//Vec3d from = mesh_obb.center() + diag;
	//Vec3d to = mesh_obb.center() - diag;
	//NURBS::NURBSCurved c = NURBS::NURBSCurved::createCurve( from, to, widget->uCount() );
	//points = m->vertex_property<Vector3>("v:point");
	//std::vector<Vec3d> mesh_points;
	//foreach(Vertex v, m->vertices()) mesh_points.push_back( points[v] );
	//basicCurveFit(c, mesh_points);
	//c.mCtrlPoint = smoothPolyline( c.mCtrlPoint, 1 );

	//graph->addNode( new Structure::Curve(c, groupID) );

	curveFit( m );

	// Clean up
	ps.clear();
	document()->setSelectedModel(entireMesh);
	//document()->removeModel(m);

	drawArea()->setRenderer(m,"Flat Wire");
	drawArea()->updateGL();
}

void nurbs_plugin::convertToSheet()
{

}

NURBS::NURBSCurved nurbs_plugin::curveFit( SurfaceMeshModel * part )
{
	NURBS::NURBSCurved fittedCurve;

	Vector3VertexProperty partPoints = part->vertex_property<Vector3>("v:point");

	double r = 0.01 * part->bbox().size().length();

	// DEBUG:
	spheres.clear();
	vs.clear();

	//foreach(Vertex v, part->vertices())
	//{
	//	Vec3d c = partPoints[v];
	//	Vec3d p = pole(c, r, part);
	//	spheres.addSphere(c,r,QColor(255,0,0,50));
	//	vs.addVector(c, p * r);
	//}

	return fittedCurve;
}

Q_EXPORT_PLUGIN (nurbs_plugin)
