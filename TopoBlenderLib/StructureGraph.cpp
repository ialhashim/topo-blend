#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QElapsedTimer>
#include <QDomElement>
#include <QStack>

#include "StructureGraph.h"
using namespace Structure;

#include "Synthesizer.h"

#include "GraphEmbed.h"
#include "GraphDraw2D.h"

#include "QuickMeshDraw.h"

Graph::Graph()
{
	property["embeded2D"] = false;
	property["showAABB"] = false;
	property["showMeshes"] = true;
}

Graph::Graph( QString fileName )
{
	property["embeded2D"] = false;

	loadFromFile( fileName );
	property["name"] = fileName;

	property["showAABB"] = false;
	property["showMeshes"] = true;
}

Graph::Graph( const Graph & other )
{
	foreach(Node * n, other.nodes)
	{
		this->addNode(n->clone());
	}

	foreach(Link * e, other.edges)
	{
		Node * n1 = this->getNode(e->n1->id);
		Node * n2 = this->getNode(e->n2->id);
		this->addEdge(n1, n2, e->coord[0], e->coord[1], e->id);
	}

	property = other.property;
	adjacency = other.adjacency;
	misc = other.misc;
}

Graph::~Graph()
{
    //qDeleteAll( nodes );
    //nodes.clear();
}

QBox3D Graph::bbox()
{
    QBox3D box;

    foreach(Node * n, nodes){
        box.unite( n->bbox().minimum() );
        box.unite( n->bbox().maximum() );
    }

	box.transform(QMatrix4x4() * 1.25);

    return box;
}

Node *Graph::addNode(Node * n)
{
    Node * found = getNode( n->id );
    if(found) return found;

    nodes.push_back(n);

	// Add property : degree and id
	n->property["degree"] = 0;
	n->property["index"] = nodes.size() - 1;

    return n;
}

Link * Graph::addEdge(QString n1_id, QString n2_id)
{
	return addEdge(getNode(n1_id), getNode(n2_id));
}

Link * Graph::addEdge(Node *n1, Node *n2)
{
    n1 = addNode(n1);
    n2 = addNode(n2);

	Vector3 intersectPoint = nodeIntersection(n1, n2);

	std::vector<Vec4d> c1,c2;

	QString edgeType = POINT_EDGE;

	if(n1->type() == SHEET && n2->type() == SHEET)
	{
		Sheet *s1 = (Sheet*) n1, *s2 = (Sheet*) n2;
		double length = qMin(s1->bbox().size().length(), s2->bbox().size().length());
		std::vector<Vec3d> pnts = s1->surface.intersect(s2->surface, 0.025 * length, c1, c2);
		
		// DEBUG:
		//foreach(Vec3d p, pnts) debugPoints3.push_back(p);

		// Fall back
		if(c1.size() < 1)
		{
			Vec3d closetPoint1 = s1->position(s1->approxCoordinates(intersectPoint));
			Vec3d closetPoint2 = s2->position(s2->approxCoordinates(intersectPoint));
			
			Vec3d delta = closetPoint2 - closetPoint1;
			Vec3d moveDelta = (1.2 * delta.norm()) * delta.normalized();
			
			s1->moveBy(moveDelta);

			std::vector<Vec3d> pnts = s1->surface.intersect(s2->surface, 0.025 * length, c1, c2);

			// DEBUG:
			//foreach(Vec3d p, pnts) debugPoints3.push_back(p);

			s1->moveBy(-moveDelta);
		}

		edgeType = LINE_EDGE;
	}
	else
	{
		c1.push_back(n1->approxCoordinates(intersectPoint));
		c2.push_back(n2->approxCoordinates(intersectPoint));
	}

    Link * e = new Link( n1, n2, c1, c2, edgeType, linkName(n1, n2) );
    edges.push_back(e);

	// Add to adjacency list
	if(adjacency.cols() != nodes.size()) adjacency = MatrixXd::Zero( nodes.size(), nodes.size() );

	adjacency(nodes.indexOf(n1), nodes.indexOf(n2)) = 1 ;
	adjacency(nodes.indexOf(n2), nodes.indexOf(n1)) = 1 ;

    return e;
}

Link * Graph::addEdge(Node *n1, Node *n2, std::vector<Vec4d> coord1, std::vector<Vec4d> coord2, QString linkName)
{
	n1 = addNode(n1);
	n2 = addNode(n2);

	if(linkName.isEmpty()) linkName = this->linkName(n1,n2);

	QString edgeType = POINT_EDGE;
	if(n1->type() == SHEET && n2->type() == SHEET) edgeType = LINE_EDGE;

	Link * e = new Link( n1, n2, coord1, coord2, edgeType, linkName );
	edges.push_back(e);

	// Add to adjacency list
	if(adjacency.cols() != nodes.size()) adjacency = MatrixXd::Zero( nodes.size(), nodes.size() );

	adjacency(nodes.indexOf(n1), nodes.indexOf(n2)) = 1 ;
	adjacency(nodes.indexOf(n2), nodes.indexOf(n1)) = 1 ;

	// Increase the degree for nodes
	int degree = n1->property["degree"].toInt();
	n1->property["degree"] = degree + 1;

	degree = n2->property["degree"].toInt();
	n2->property["degree"] = degree + 1;

	return e;
}

void Graph::removeEdge( Node * n1, Node * n2 )
{
	int edge_idx = -1;

	for(int i = 0; i < (int)edges.size(); i++)
	{
		Link * e = edges[i];

		if((e->n1 == n1 && e->n2 == n2) || (e->n2 == n1 && e->n1 == n2)){
			edge_idx = i;
			break;
		}
	}

	if(edge_idx < 0) return;

	edges.remove(edge_idx);

	adjacency(nodes.indexOf(n1), nodes.indexOf(n2)) = 0;
	adjacency(nodes.indexOf(n2), nodes.indexOf(n1)) = 0;
}

QString Graph::linkName(Node *n1, Node *n2)
{
    return QString("%1 : %2").arg(n1->id).arg(n2->id);
}

QString Graph::linkName( QString n1_id, QString n2_id )
{
	return linkName(getNode(n1_id),getNode(n2_id));
}

Node *Graph::getNode(QString nodeID)
{
    foreach(Node* n, nodes)
	{
        if(n->id == nodeID) 
			return n;
	}

    return NULL;
}

Link *Graph::getEdge(QString id1, QString id2)
{
	for(int i = 0; i < (int)edges.size(); i++)
	{
		Link * e = edges[i];

		QString nid1 = e->n1->id;
		QString nid2 = e->n2->id;

		if( (nid1 == id1 && nid2 == id2) || (nid1 == id2 && nid2 == id1))
			return e;
	}
	
	return NULL;
}

void Graph::draw()
{
	vs.draw();
	ps.draw();

    foreach(Node * n, nodes)
    {
        n->draw();

		if(n->type() == SHEET)
		{
			Sheet * s = (Sheet*) n;

			glPointSize(3);
			glDisable(GL_LIGHTING);

			glColor3d(0,1,0);
			glBegin(GL_POINTS);
			foreach(Vector3 p, s->surface.debugPoints)	glVector3(p);
			glEnd();

			//glBegin(GL_TRIANGLES);
			//foreach(std::vector<Vector3> tri, s->surface.generateSurfaceTris(0.4))
			//{
			//	glColor3d(1,0,0); glVector3(tri[0]);
			//	glColor3d(0,1,0); glVector3(tri[1]);
			//	glColor3d(0,0,1); glVector3(tri[2]);
			//}
			//glEnd();
		}

		if(n->property.contains("samples"))
		{
			QVector<Vec3d> points, normals;

			if(!n->property.contains("cached_points"))
			{
				// Without blending!
				if(n->type() == Structure::CURVE)
				{
					Structure::Curve * curve = (Structure::Curve *)n;
					Synthesizer::blendGeometryCurves(curve,curve,0,points,normals);
				}
				if(n->type() == Structure::SHEET)
				{
					Structure::Sheet * sheet = (Structure::Sheet *)n;
					Synthesizer::blendGeometrySheets(sheet,sheet,0,points,normals);
				}

				n->property["cached_points"].setValue(points);
				n->property["cached_normals"].setValue(normals);
			}
			else
			{
				points = n->property["cached_points"].value< QVector<Vec3d> >();
				normals = n->property["cached_normals"].value< QVector<Vec3d> >();
			}

			glPointSize(3.0);
			glEnable(GL_LIGHTING);
			glBegin(GL_POINTS);
			for(int i = 0; i < (int)points.size(); i++){
				glNormal3(normals[i]);
				glVector3(points[i]);
			}
			glEnd();
		}
		else
		{
			// Draw node mesh
			if( property["showMeshes"].toBool() )
			{
				if(n->property.contains("mesh"))
				{
					QuickMeshDraw::drawMeshWireFrame( n->property["mesh"].value<SurfaceMeshModel*>() );
				}
			}
		}
    }

    foreach(Link * e, edges)
    {
        e->draw();
    }

	glDisable(GL_LIGHTING);
	glColor3d(1,1,0); glPointSize(20);
	glBegin(GL_POINTS); foreach(Vector3 p, debugPoints) glVector3(p); glEnd();

	glColor3d(0,1,1); glPointSize(30);
	glBegin(GL_POINTS); foreach(Vector3 p, debugPoints2) glVector3(p); glEnd();

	glColor3d(1,0,1); glPointSize(40);
	glBegin(GL_POINTS); foreach(Vector3 p, debugPoints3) glVector3(p); glEnd();
	glEnable(GL_LIGHTING);

	// Materialized
	glBegin(GL_QUADS);
	foreach(QuadFace f, cached_mesh.faces)
	{
		glNormal3(cross(cached_mesh.points[f[1]]-cached_mesh.points[f[0]],cached_mesh.points[f[2]]-cached_mesh.points[f[0]]).normalized());
		glVector3(cached_mesh.points[f[0]]);
		glVector3(cached_mesh.points[f[1]]);
		glVector3(cached_mesh.points[f[2]]);
		glVector3(cached_mesh.points[f[3]]);
	}
	glEnd();

	glDisable(GL_LIGHTING);
	glPointSize(15);
	glColor3d(1,1,0);
	glBegin(GL_POINTS);
	foreach(Vector3 p, cached_mesh.debug) glVector3(p);
	glEnd();
	glEnable(GL_LIGHTING);

	// AABB
	if (property["showAABB"].toBool()) drawAABB();
}


void Graph::drawAABB()
{
	if (!property.contains("AABB"))
		property["AABB"].setValue(bbox());
	QBox3D aabb = property["AABB"].value<QBox3D>();
	QVector3D qc = aabb.center();
	QVector3D diagonal = aabb.maximum() - aabb.minimum();

	double width = diagonal.x()/2;
	double length = diagonal.y()/2;
	double height = diagonal.z()/2;

	Vec3d center(qc.x(), qc.y(), qc.z());
	Vec3d  c1, c2, c3, c4;
	Vec3d  bc1, bc2, bc3, bc4;

	c1 = Vec3d (width, length, height) + center;
	c2 = Vec3d (-width, length, height) + center;
	c3 = Vec3d (-width, -length, height) + center;
	c4 = Vec3d (width, -length, height) + center;

	bc1 = Vec3d (width, length, -height) + center;
	bc2 = Vec3d (-width, length, -height) + center;
	bc3 = Vec3d (-width, -length, -height) + center;
	bc4 = Vec3d (width, -length, -height) + center;

	glDisable(GL_LIGHTING);
	glLineWidth(1.0f);

	glBegin(GL_LINES);
	glVertex3dv(c1);glVertex3dv(bc1);
	glVertex3dv(c2);glVertex3dv(bc2);
	glVertex3dv(c3);glVertex3dv(bc3);
	glVertex3dv(c4);glVertex3dv(bc4);
	glVertex3dv(c1);glVertex3dv(c2);
	glVertex3dv(c3);glVertex3dv(c4);
	glVertex3dv(c1);glVertex3dv(c4);
	glVertex3dv(c2);glVertex3dv(c3);
	glVertex3dv(bc1);glVertex3dv(bc2);
	glVertex3dv(bc3);glVertex3dv(bc4);
	glVertex3dv(bc1);glVertex3dv(bc4);
	glVertex3dv(bc2);glVertex3dv(bc3);
	glEnd();

	glEnable(GL_LIGHTING);

}



void Graph::draw2D(int width, int height)
{
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

    // Embed structure graph to 2D screen plane
    if(!property["embeded2D"].toBool())
	{
        //GraphEmbed::embed( this );
		GraphEmbed::circleEmbed( this );

		fontImage = QImage(":/images/font.png");
	}

	// Find 2D bounds
	QBox3D bound2D;
	foreach(Node * n, nodes){
		QVector3D center = n->vis_property[NODE_CENTER].value<QVector3D>();
		bound2D.unite(center);
	}
	Scalar boundRadius = 0.5 * bound2D.size().length();

	QVector3D minBound = bound2D.minimum();
	QVector3D corner = QVector3D(qMin(0.0, minBound.x()), qMin(0.0, minBound.y()), qMin(0.0, minBound.z()));
	QVector3D graph_center = 0.5 * QVector3D(width, height,0);
	QVector3D scaling = QVector3D(width, height, 0) / boundRadius;

	// Compute 2D node centers
	QMap<int, Node*> nmap;
	std::vector<Vector3> node_centers;

	foreach(Node * n, nodes){
		QVector3D center = graph_center + (scaling * (n->vis_property[NODE_CENTER].value<QVector3D>() - corner));
		node_centers.push_back(center);
		nmap[nmap.size()] = n;
	}

	int N = nodes.size();

	// Draw edges:
	glColorQt(QColor(0,0,0));
	foreach(Link * e, edges)
	{
		Vector3 c1 = node_centers[nmap.key(e->n1)];
		Vector3 c2 = node_centers[nmap.key(e->n2)];
		drawLine(Vec2i(c1.x(), c1.y()), Vec2i(c2.x(), c2.y()), 1);
	}

	// Draw nodes:
	for(int i = 0; i < N; i++)
	{
		glColorQt( (nmap[i]->type() == SHEET ? QColor(50,0,0) : QColor(0,50,0)) );
		Vector3 c = node_centers[i];

		int w = stringWidthGL(qPrintable(nmap[i]->id)) * 1.1;
		int h = stringHeightGL();

		drawRect(Vec2i(c.x(), c.y()), w, h);
	}

	// Draw titles:
	glColorQt(QColor(255,255,255));
	beginTextDraw(fontImage);
	for(int i = 0; i < N; i++){
		Vector3 c = node_centers[i];

		int w = stringWidthGL(qPrintable(nmap[i]->id)) * 1.1;
		int h = stringHeightGL();

		drawStringGL( c.x() - (0.5 * w), c.y() - (0.5 * h), qPrintable(nmap[i]->id) );
	}
	endTextDraw();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
}

void Graph::saveToFile( QString fileName ) const
{
	if(nodes.size() < 1) return;

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QTextStream out(&file);
	out << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n<document>\n";

	// Save nodes
	foreach(Node * n, nodes)
	{
		out << "<node>\n";

		// Type and ID
		out << QString("\t<id>%1</id>\n").arg(n->id);
		out << QString("\t<type>%1</type>\n").arg(n->type());
		out << QString("\t<mesh>%1</mesh>\n\n").arg(n->property["mesh_filename"].toString()); // relative

		// Control count
		out << "\t<controls>\n";
		std::vector<int> controlCount = n->controlCount();
		foreach(int c, controlCount)
			out << QString("\t\t<c>%1</c>\n").arg(c);
		out << "\t</controls>\n\n";

		// Control Points
		foreach(Vec3d p, n->controlPoints())
			out << QString("\t<point>%1 %2 %3</point>\n").arg(p.x()).arg(p.y()).arg(p.z());

		// Weights
		out << "\n\t<weights>";
		foreach(Scalar w, n->controlWeights())
			out << QString::number(w) + " ";
		out << "</weights>\n";

		out << "</node>\n\n";
	}

	// Save edges
	foreach(Link * e, edges)
	{
		out << "<edge>\n";
		out << QString("\t<id>%1</id>\n").arg(e->id);
		out << QString("\t<type>%1</type>\n\n").arg(e->type);
		out << QString("\t<n>%1</n>\n").arg(e->n1->id);
		out << QString("\t<n>%1</n>\n").arg(e->n2->id);

		for(int k = 0; k < 2; k++)
		{
			out << "\t<coord>\n";
			foreach(Vec4d c, e->coord[k])
				out << "\t\t<uv>" << c[0] << " " << c[1] << " " << c[2] << " " << c[3] << "</uv>\n";
			out << "\t</coord>\n";
		}

		out << "</edge>\n\n";
	}

	out << "\n</document>\n";
	file.close();
}

void Graph::loadFromFile( QString fileName )
{
	// Clear data
	nodes.clear();
	edges.clear();

	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QDomDocument mDocument;
	mDocument.setContent(&file, false);    
	
	int degree = 3;

	// For each node
	QDomNodeList node_list = mDocument.firstChildElement("document").elementsByTagName("node");
	int num_nodes = node_list.count();

	for(int i = 0; i < num_nodes; i++)
	{
		QDomNode node = node_list.at(i);

		QString id = node.firstChildElement("id").text();
		QString node_type = node.firstChildElement("type").text();
		QString mesh_filename = node.firstChildElement("mesh").text();
		
		// Find number of control elements
		QVector<int> control_count;
		QDomNodeList controls_list = node.firstChildElement("controls").toElement().elementsByTagName("c");
		for(int c = 0; c < (int)controls_list.size(); c++)
			control_count.push_back( controls_list.at(c).toElement().text().toInt() );

		// Load all control points
		std::vector<Vec3d> ctrlPoints;
		QDomNode n = node.firstChildElement("point");
		while (!n.isNull()) {
			if (n.isElement()) {
				QStringList point = n.toElement().text().split(" ");
				ctrlPoints.push_back( Vec3d(point[0].toDouble(),point[1].toDouble(),point[2].toDouble()) );
			}
			n = n.nextSiblingElement("point");
		}

		// Load all control weights
		std::vector<Scalar> ctrlWeights;
		QStringList weights = node.firstChildElement("weights").toElement().text().trimmed().split(" ");
		foreach(QString w, weights) ctrlWeights.push_back(w.toDouble());

		// Add node
		Structure::Node * new_node = NULL;

		if(node_type == CURVE)
		{
			new_node = addNode( new Curve( NURBSCurve(ctrlPoints, ctrlWeights, degree, false, true), id) );
		}
		else if(node_type == SHEET)
		{
			if(control_count.size() < 2) continue;

			std::vector< std::vector<Vec3d> > cp = std::vector< std::vector<Vec3d> > (control_count.first(), std::vector<Vec3d>(control_count.last(), Vector3(0)));
			std::vector< std::vector<Scalar> > cw = std::vector< std::vector<Scalar> > (control_count.first(), std::vector<Scalar>(control_count.last(), 1.0));

			for(int u = 0; u < control_count.first(); u++)
			{
				for(int v = 0; v < control_count.last(); v++)
				{
					int idx = (u * control_count.last()) + v;
					cp[u][v] = ctrlPoints[idx];
					cw[u][v] = ctrlWeights[idx];
				}
			}

			new_node = addNode( new Sheet( NURBSRectangle(cp, cw, degree, degree, false, false, true, true), id ) );
		}

		// Mesh file path
		new_node->property["mesh_filename"].setValue( mesh_filename );
		QDir::setCurrent( fileInfo.dir().path() );
		QFile mfile(mesh_filename);

		// Load node's mesh and make it ready
		if (mfile.exists())
		{
			SurfaceMeshModel * nodeMesh = new SurfaceMeshModel(mesh_filename, id);
			nodeMesh->read( qPrintable(mesh_filename) );
			nodeMesh->update_face_normals();
			nodeMesh->update_vertex_normals();
			nodeMesh->updateBoundingBox();
			new_node->property["mesh"].setValue(nodeMesh);
		}
	}

	// For each edge
	QDomNodeList edge_list = mDocument.firstChildElement("document").elementsByTagName("edge");
	int num_edges = edge_list.count();

	for(int i = 0; i < num_edges; i++)
	{
		QDomNode edge = edge_list.at(i);

		QString id = edge.firstChildElement("id").text();
		QString edge_type = edge.firstChildElement("type").text();

		QDomNodeList n = edge.toElement().elementsByTagName("n");
		
		QString n1_id = n.at(0).toElement().text();
		QString n2_id = n.at(1).toElement().text();

		QDomNodeList coordList = edge.toElement().elementsByTagName("coord");
		std::vector< std::vector<Vec4d> > coords((int)coordList.size());
		for(int j = 0; j < (int) coords.size(); j++)
		{
			QDomNodeList uv = coordList.at(j).toElement().elementsByTagName("uv");
			int uv_count = uv.count();

			for(int k = 0; k < uv_count; k++)
			{
				QStringList c = uv.at(k).toElement().text().split(" ");
				coords[j].push_back(Vec4d(c[0].toDouble(), c[1].toDouble(), c[2].toDouble(), c[3].toDouble()));
			}
		}

		addEdge(getNode(n1_id), getNode(n2_id), coords.front(), coords.back(), id);
	}

	file.close();
}

void Graph::materialize( SurfaceMeshModel * m, Scalar voxel_scaling )
{
	QElapsedTimer timer; timer.start();

	cached_mesh.clear();

	QBox3D box = bbox();
	QVector3D b = bbox().size();
	Scalar avg = (b.x() + b.y() + b.z()) / 3.0;
	Scalar voxel_size = (avg / 70) * voxel_scaling;

	DynamicVoxel vox( voxel_size );

	Vector3 half_voxel = -0.5 * Vector3( voxel_size );

	vox.begin();

	foreach(Node * n, nodes)
	{
		QElapsedTimer nodoe_timer; nodoe_timer.start();

		std::vector< std::vector<Vector3> > parts = n->discretized( voxel_size );
		int c = parts.front().size();

		// Curve segments
		if(c == 2)
		{
			foreach(std::vector<Vector3> segment, parts)
			{
				vox.addCapsule( segment.front() + half_voxel, segment.back() + half_voxel, voxel_size * 2 );
			}
		}

		// Sheet triangles
		if(c == 3)
		{
			foreach(std::vector<Vector3> segment, parts)
			{
				QBox3D triBox;
				foreach(Vector3 p, segment) triBox.unite(p + half_voxel);
				vox.addBox( triBox.minimum(), triBox.maximum() );
			}
		}

		qDebug() << "Node built [" << n->id << "] " << nodoe_timer.elapsed() << " ms";
	}

	vox.end();

	qDebug() << "Voxels built " << timer.elapsed() << " ms";

	vox.buildMesh(m, cached_mesh);

    if(m) cached_mesh.clear();
}

Node *Graph::rootBySize()
{
	int idx = 0;
	Scalar maxArea = 0;

	for(int i = 0; i < nodes.size(); i++){
		Vector3 diagonal = nodes[i]->bbox().size();

		double area = (diagonal[0] == 0 ? 1 : diagonal[0]) * 
					  (diagonal[1] == 0 ? 1 : diagonal[1]) * 
					  (diagonal[2] == 0 ? 1 : diagonal[2]);

		if( area > maxArea){
			idx = i;
			maxArea = area;
		}
	}

    return nodes[idx];
}

Node *Graph::rootByValence()
{
	int maxIdx = 0;
	int maxValence = valence(nodes[maxIdx]);

	for(int i = 0; i < (int)nodes.size(); i++)
	{
		int curVal = valence(nodes[i]);
		if(curVal > maxValence)
		{
			maxValence = curVal;
			maxIdx = i;
		}
	}

    return nodes[maxIdx];
}

SurfaceMeshTypes::Vector3 Graph::nodeIntersection( Node * n1, Node * n2 )
{
	double s1 = n1->bbox().size().length();
	double s2 = n2->bbox().size().length();

	Scalar r = 0.04 * qMin(s1, s2);

	if(n1->type() == Structure::SHEET && n2->type() == Structure::SHEET)
		r *= 2;

	std::vector< std::vector<Vector3> > parts1 = n1->discretized(r);
	std::vector< std::vector<Vector3> > parts2 = n2->discretized(r);

	Scalar minDist = DBL_MAX;
	int minI = 0, minJ = 0;
	int minIm = 0, minJn = 0;

	for(int i = 0; i < (int)parts1.size(); i++)
	{
		for(int j = 0; j < (int)parts2.size(); j++)
		{
			Scalar local_min_dis = DBL_MAX;
			int min_m = 0;
			int min_n = 0;

			for (int m = 0; m < (int)parts1[i].size(); m++)
			{
				for (int n = 0;  n < (int)parts2[j].size(); n++ )
				{
					double dis = (parts1[i][m] - parts2[j][n]).norm();
					if (dis < local_min_dis)
					{
						local_min_dis = dis;
						min_m = m;
						min_n = n;
					}
				}
			}

			if (local_min_dis < minDist)
			{
				minDist = local_min_dis;
				minI = i;
				minJ = j;

				minIm = min_m;
				minJn = min_n;
			}

		}
	}

	Vector3 p1 = parts1[minI][minIm];
	Vector3 p2 = parts2[minJ][minJn];
	return (p1 + p2) / 2;

	//// Compare parts bounding boxes
	//for(int i = 0; i < (int)parts1.size(); i++)
	//{
	//	Vector3 mean1(0);
	//	Scalar r1 = 0;

	//	foreach(Vector3 p, parts1[i])	mean1 += p;
	//	mean1 /= parts1[i].size();
	//	foreach(Vector3 p, parts1[i])	r1 = qMax((p - mean1).norm(), r1);

	//	for(int j = 0; j < (int)parts2.size(); j++)
	//	{
	//		Vector3 mean2(0);
	//		Scalar r2 = 0;

	//		foreach(Vector3 p, parts2[j])	mean2 += p;
	//		mean2 /= parts2[j].size();
	//		foreach(Vector3 p, parts2[j])	r2 = qMax((p - mean2).norm(), r2);

	//		Vector3 diff = mean1 - mean2;
	//		Scalar dist = diff.norm();

	//		Scalar sphereDist = dist - r1 - r2;

	//		if(sphereDist <= 0)
	//		{
	//			std::vector<Vector3> p1 = parts1[ i ];
	//			std::vector<Vector3> p2 = parts2[ j ];
	//			int g1 = p1.size(), g2 = p2.size();
	//			Vector3 c1,c2;
	//			Scalar s,t;

	//			if(g1 == 2 && g2 == 2) ClosestPointSegments		(p1[0],p1[1],  p2[0],p2[1],       s, t, c1, c2);	// curve -- curve
	//			if(g1 == 2 && g2 == 3) ClosestSegmentTriangle	(p1[0],p1[1],  p2[0],p2[1],p2[2],       c1, c2);	// curve -- sheet
	//			if(g1 == 3 && g2 == 2) ClosestSegmentTriangle	(p2[0],p2[1],  p1[0],p1[1],p1[2],       c1, c2);	// sheet -- curve
	//			if(g1 == 3 && g2 == 3) TriTriIntersect			(p1[0],p1[1],p1[2],  p2[0],p2[1],p2[2], c1, c2);	// sheet -- sheet
	//			
	//			Scalar dist = (c1 - c2).norm();

	//			if(dist < minDist)
	//			{
	//				minI = i;
	//				minJ = j;
	//				minDist = dist;
	//			}

	//			//foreach(Vector3 w, parts1[i]) debugPoints2.push_back(w);
	//			//foreach(Vector3 w, parts2[j]) debugPoints2.push_back(w);
	//		}
	//	}
	//}

	//// Find the exact intersection point
	//std::vector<Vector3> p1 = parts1[ minI ];
	//std::vector<Vector3> p2 = parts2[ minJ ];
	//int g1 = p1.size(), g2 = p2.size();

	//Vector3 c1,c2;
	//Scalar s,t;

	//if(g1 == 2 && g2 == 2) ClosestPointSegments		(p1[0],p1[1],  p2[0],p2[1],       s, t, c1, c2);	// curve -- curve
	//if(g1 == 2 && g2 == 3) ClosestSegmentTriangle	(p1[0],p1[1],  p2[0],p2[1],p2[2],       c1, c2);	// curve -- sheet
	//if(g1 == 3 && g2 == 2) ClosestSegmentTriangle	(p2[0],p2[1],  p1[0],p1[1],p1[2],       c1, c2);	// sheet -- curve
	//if(g1 == 3 && g2 == 3) TriTriIntersect			(p1[0],p1[1],p1[2],  p2[0],p2[1],p2[2], c1, c2);	// sheet -- sheet

	//debugPoints.push_back(c1);
	//debugPoints.push_back(c2);
	//foreach(Vector3 w, p1) debugPoints2.push_back(w);
	//foreach(Vector3 w, p2) debugPoints3.push_back(w);

	//return (c1 + c2) / 2.0;
}

void Graph::printAdjacency()
{
	int n = nodes.size();

	qDebug() << "\n\n== Adjacency Matrix ==";

	for(int i = 0; i < n; i++)
	{
		QString line;

		for(int j = 0; j < n; j++)
			line += QString(" %1 ").arg(adjacency(i,j));

		qDebug() << line;
	}

	qDebug() << "======";
}

int Graph::valence( Node * n )
{
	int idx = nodes.indexOf(n);
	return 0.5 * (adjacency.col(idx).sum() + adjacency.row(idx).sum());
}

Curve* Graph::getCurve( Link * l )
{
	Structure::Node *n1 = l->n1, *n2 = l->n2;
	return (Structure::Curve *) ((n1->type() == Structure::CURVE) ? n1: n2);
}

QVector<Link*> Graph::getEdges( QString nodeID )
{
	QVector<Link*> mylinks;

	for(int i = 0; i < edges.size(); i++)
	{
		Link * l = edges[i];
		if( l->hasNode(nodeID) ) mylinks.push_back(l);
	}

	return mylinks;
}

QMap<Link*, std::vector<Vec4d> > Graph::linksCoords( QString nodeID )
{
    QMap< Link*, std::vector<Vec4d> > coords;

	for(int i = 0; i < edges.size(); i++)
	{
		Link * l = edges[i];
		if(!l->hasNode(nodeID)) continue;

		coords[l] = l->getCoord(nodeID);
	}

	return coords;
}

QVector<Link*> Graph::nodeEdges( QString nodeID )
{
	QVector<Link*> nodeLinks;
	
	foreach(Link * e, edges) if(e->hasNode(nodeID)) 
		nodeLinks.push_back(e);

	return nodeLinks;
}

Node * Graph::removeNode( QString nodeID )
{
	Structure::Node * n = getNode(nodeID);

	foreach(Link * e, getEdges(nodeID)){
		edges.remove(edges.indexOf(e));
	}

	nodes.remove(nodes.indexOf(n));

	return n;
}

QList<Link*> Graph::furthermostEdges( QString nodeID )
{
	QMap<double, Link*> sortedLinks;
	foreach(Link * e, nodeEdges(nodeID))
	{
		foreach(Vec4d c, e->getCoord(nodeID))
			sortedLinks[c.norm()] = e;
	}

	return sortedLinks.values();
}

bool Graph::isCutNode( QString nodeID )
{
	QSet<Link*> isDeleted;

	QMap<QString, bool> visitedNodes;
	QStack<QString> nodesToVisit;

	foreach(Link * l, getEdges(nodeID))
	{
		if(nodesToVisit.size() == 0) 
			nodesToVisit.push(l->otherNode(nodeID)->id);
		isDeleted.insert(l);
	}

	// Visit all nodes without going through a deleted edge
	while(! nodesToVisit.isEmpty() )
	{
		QString id = nodesToVisit.pop();
		visitedNodes[id] = true;

		foreach(Link * l, getEdges(id))
		{
			if(isDeleted.contains(l)) continue;

			QString adjID = l->otherNode(id)->id;

			if( !visitedNodes.contains(adjID) && !nodesToVisit.contains(adjID))
				nodesToVisit.push(adjID);
		}
	}

	if(visitedNodes.size() != nodes.size() - 1)
		return true;
	else
		return false;
}

void Graph::replaceCoords( QString nodeA, QString nodeB, std::vector<Vec4d> coordA, std::vector<Vec4d> coordB )
{
	// Get shared link
	Link * sharedLink = getEdge(nodeA, nodeB);

	if(!sharedLink) return;

	sharedLink->setCoord(nodeA, coordA);
	sharedLink->setCoord(nodeB, coordB);
}

Vector3 Graph::position( QString nodeID, Vec4d coord )
{
	return getNode(nodeID)->position(coord);
}

void Graph::moveBottomCenterToOrigin()
{
	QBox3D aabb = bbox();
	double height = aabb.maximum().z() - aabb.minimum().z();
	Vec3d bottom_center(aabb.center().x(), aabb.center().y(), aabb.center().z() - height/2);

	foreach (Structure::Node * node, nodes)
	{
		node->moveBy( -bottom_center );
		
		// Move actual geometry
		SurfaceMeshModel* model = node->property["mesh"].value<SurfaceMeshModel*>();
		Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
		foreach(Vertex v, model->vertices())
			points[v] -= bottom_center;
	}

	// Update the bounding box
	property["AABB"].setValue(bbox());
}

void Graph::normalize()
{
	// Normalize the height to be 1
	QBox3D aabb = bbox();
	double height = aabb.maximum().z() - aabb.minimum().z();
	double scaleFactor = 1.0 / height;

	foreach (Structure::Node * node, nodes)
	{
		node->scale(scaleFactor);
		
		// Move actual geometry
		SurfaceMeshModel* model = node->property["mesh"].value<SurfaceMeshModel*>();
		Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
		foreach(Vertex v, model->vertices())
			points[v] *= scaleFactor;
	}

	// Update the bounding box
	property["AABB"].setValue(bbox());
}

void Graph::rotate( double angle, Vector3 axis )
{
	foreach (Structure::Node * node, nodes)
	{
		node->rotate(angle, axis);

		// Move actual geometry
		SurfaceMeshModel* model = node->property["mesh"].value<SurfaceMeshModel*>();
		Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
		model->updateBoundingBox();

		angle *= 3.14159265358979 /180; 

		foreach(Vertex v, model->vertices())
		{
			points[v] = rotatedVec(points[v], angle, axis);
		}
	}
}

void Graph::removeEdge( QString n1_id, QString n2_id )
{
	removeEdge( getNode(n1_id),getNode(n2_id) );
}

void Graph::scale( double scaleFactor )
{
	double current_scale = 1.0;
	if (property.contains("scale"))
		current_scale = property["scale"].toDouble();

	double relative_scale = scaleFactor / current_scale;

	foreach (Structure::Node * node, nodes)
	{
		node->scale(relative_scale);

		// Move actual geometry
		SurfaceMeshModel* model = node->property["mesh"].value<SurfaceMeshModel*>();
		Vector3VertexProperty points = model->vertex_property<Vec3d>("v:point");
		foreach(Vertex v, model->vertices())
			points[v] *= relative_scale;
	}

	property["scale"] = scaleFactor;

	property["AABB"].setValue(bbox());
}

QVector<POINT_ID> Graph::selectedControlPointsByColor(QColor color)
{
	QVector<POINT_ID> result;
	for (int nID = 0; nID < (int)nodes.size(); nID++)
	{
		Structure::Node *node = nodes[nID];
		foreach (int pID, node->selections.keys())
		{
			if (node->selections[pID] == color)
				result.push_back(std::make_pair(nID, pID));
		}
	}

	return result;
}

void Structure::Graph::clearSelections()
{
	foreach(Structure::Node *node, nodes)
		node->selections.clear();
}

void Structure::Graph::printLinksInfo()
{
	foreach(Link * e, edges)
	{
		QStringList c_string;

		foreach(Array1D_Vec4d coords, e->coord)
		{
			c_string << "";
			foreach(Vec4d c, coords)
				c_string.back() += QString("|%1  %2|").arg(c[0]).arg(c[1]);
		}

		qDebug() << e->id << "\t=> c1 [ " << c_string.front() << " ] \t c2 [ " << c_string.back() << " ]" ;
	}
}
