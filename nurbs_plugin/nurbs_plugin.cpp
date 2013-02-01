#include <QFileDialog>
#include <QStack>

#include "nurbs_plugin_global.h"

#include "nurbs_plugin.h"
#include "NanoKdTree.h"
#include "StructureGraph.h"

#include "interfaces/ModePluginDockWidget.h"

#include "LSCM.h"
#include "BoundaryFitting.h"

#include "../CustomDrawObjects.h"

void nurbs_plugin::create()
{
    ModePluginDockWidget * dockwidget = new ModePluginDockWidget(mainWindow());
    widget = new NURBSTools(this);
    dockwidget->setWidget(widget);
    dockwidget->setWindowTitle(widget->windowTitle());
    mainWindow()->addDockWidget(Qt::RightDockWidgetArea,dockwidget);

    points = mesh()->vertex_property<Vector3>("v:point");
}

void nurbs_plugin::decorate()
{
	// Draw OBB
	mesh_obb.draw();

	for(int i = 0; i < (int)curves.size(); i++)
	{
		NURBS::CurveDraw::draw( &curves[i], QColor(0,255,255), true );
	}

	for(int i = 0; i < (int)rects.size(); i++)
	{
		NURBS::SurfaceDraw::draw( &rects[i], QColor(0,255,255), true );
	}
}

void nurbs_plugin::doFitCurve()
{
    qDebug() << "Curve fitting..";

	mesh_obb = OBB_Volume( mesh() );

	std::vector<Vec3d> corners = mesh_obb.corners();

	Vector3 from( corners.front() );
	Vector3 to( corners.back() );

    NURBS::NURBSCurved c = NURBS::NURBSCurved::createCurve( from, to, widget->uCount() );

	std::vector<Vec3d> mesh_points;
	foreach(Vertex v, mesh()->vertices()) mesh_points.push_back( points[v] );

	basicCurveFit(c, mesh_points);

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
	// Pick a side by clustering normals

	// 1) Find edge with flat dihedral angle
	double angleThreshold = deg_to_rad( 10.0 );

	Halfedge startEdge = mesh()->halfedges_begin();
	foreach(Halfedge h, mesh()->halfedges()){
		if(calc_dihedral_angle( mesh(), h ) < angleThreshold){
			startEdge = h;
			break;
		}
	}

	// 2) Grow region by comparing difference of adjacent dihedral angles
	SurfaceMeshModel::Vertex_property<bool> vvisited = mesh()->add_vertex_property<bool>("v:visited", false);

	QStack<SurfaceMeshModel::Vertex> to_visit;
	to_visit.push( mesh()->to_vertex(startEdge) );

	while(!to_visit.empty())
	{
		Vertex cur_v = to_visit.pop();
		if( vvisited[cur_v] ) continue;
		vvisited[cur_v] = true;

		// Sum of angles around
		double sumAngles = 0.0;
		foreach(Halfedge hj, mesh()->onering_hedges(cur_v)){
			sumAngles += abs(calc_dihedral_angle( mesh(), hj ));
		}

		foreach(Halfedge hj, mesh()->onering_hedges(cur_v))
		{
			Vertex vj = mesh()->to_vertex(hj);
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
		foreach(Vertex v, mesh()->vertices())
			if(vvisited[v]) all_points.insert(v);

		border.clear();
		foreach(Vertex v, mesh()->vertices()){
			if(vvisited[v]){
				foreach(Halfedge hj, mesh()->onering_hedges(v)){
					Vertex vj = mesh()->to_vertex(hj);
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
			foreach(Halfedge hj, mesh()->onering_hedges(vv))
				vvisited[ mesh()->to_vertex(hj) ] = false;
		}
	}

	SurfaceMeshModel * submesh = NULL;
	
	bool isOpen = false;
	foreach(Vertex v, inner){
		if(mesh()->is_boundary(v)){
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
			foreach(Halfedge hj, mesh()->onering_hedges(v)){
				Face f = mesh()->face(hj);
				innerFaces.insert(f);
				Surface_mesh::Vertex_around_face_circulator vit = mesh()->vertices(f),vend=vit;
				do{ inFacesVerts.insert( Vertex(vit) ); } while(++vit != vend);
			}
		}

		// Create sub-mesh
		submesh = new SurfaceMeshModel("SideFlat.obj","SideFlat");
	
		// Add vertices
		std::map<Vertex,Vertex> vmap;
		foreach(Vertex v, inFacesVerts){
			vmap[ v ] = Vertex(vmap.size());
			submesh->add_vertex( points[v] );
		}

		// Add faces
		foreach(Face f, innerFaces){
			std::vector<Vertex> verts;
			Surface_mesh::Vertex_around_face_circulator vit = mesh()->vertices(f),vend=vit;
			do{ verts.push_back( Vertex(vit) ); } while(++vit != vend);
			submesh->add_triangle( vmap[verts[0]], vmap[verts[1]], vmap[verts[2]] );
		}
	}
	else
	{
		submesh = mesh();
	}

	Vector3VertexProperty pos_submesh = submesh->vertex_property<Vector3>(VPOINT);

	if(mesh() != submesh)
		document()->addModel(submesh);

	// Find 4 corners of a NURBS rect on mesh boundary
	BoundaryFitting bf( submesh );

	PointSoup * ps = new PointSoup;
	foreach(Vertex v, submesh->vertices())
		ps->addPoint( pos_submesh[v], qtJetColorMap(1.0 - bf.dists[v]) );
	drawArea()->addRenderObject(ps);

	// Construct rectangle


	drawArea()->updateGL();
}

void nurbs_plugin::clearAll()
{
	curves.clear();
	rects.clear();
}

void nurbs_plugin::saveAll()
{
	Structure::Graph g;

	foreach(NURBS::NURBSCurved curve, curves)
	{
		g.addNode( new Structure::Curve(curve, mesh()->name) );
	}

	foreach(NURBS::NURBSRectangled rect, rects)
	{
		g.addNode( new Structure::Sheet(rect, mesh()->name) );
	}

	QString filename = QFileDialog::getSaveFileName(0, tr("Save Model"), 
		mainWindow()->settings()->getString("lastUsedDirectory"), tr("Model Files (*.xml)"));

	g.saveToFile(filename);
}

void nurbs_plugin::doFitSurface_old()
{
    qDebug() << "Surface fitting..";

	mesh_obb = OBB_Volume( mesh() );

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

Q_EXPORT_PLUGIN (nurbs_plugin)
