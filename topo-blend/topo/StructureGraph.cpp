#include <QFile>
#include <QElapsedTimer>
#include <QDomElement>

#include "StructureGraph.h"
using namespace Structure;

#include "GraphEmbed.h"
#include "GraphDraw2D.h"

Graph::Graph()
{
	property["embeded2D"] = false;
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

    return box;
}

Node *Graph::addNode(Node * n)
{
    Node * found = getNode( n->id );
    if(found) return found;

    nodes.insert(n);
    return n;
}

Link Graph::addEdge(Node *n1, Node *n2)
{
    if(n2->valence() > n1->valence())
        std::swap(n1, n2);

    n1 = addNode(n1);
    n2 = addNode(n2);

	Vector3 intersectPoint = nodeIntersection(n1, n2);

	//debugPoints.push_back(intersectPoint);

	Vector3 c1 = n1->approxProjection(intersectPoint);
	Vector3 c2 = n2->approxProjection(intersectPoint);

    Link e( n1, n2, c1, c2, "none", linkName(n1, n2) );
    edges.insert(e);

    return e;
}

QString Graph::linkName(Node *n1, Node *n2)
{
    return QString("%1 : %2").arg(n1->id).arg(n2->id);
}

Node *Graph::getNode(QString nodeID)
{
    foreach(Node* n, nodes)
        if(n->id == nodeID) return n;

    return NULL;
}

void Graph::draw()
{
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
			//foreach(Vector3 p, s->surface.debugPoints)	glVector3(p);
			glEnd();
		}
    }

    foreach(Link e, edges)
    {
        e.draw();
    }

	// DEBUG:
	glDisable(GL_LIGHTING);
	glPointSize(20);
	glBegin(GL_POINTS);
	glColor3d(1,1,0);	foreach(Vector3 p, debugPoints2)	glVector3(p);
	glColor3d(1,0,1);	foreach(Vector3 p, debugPoints3)	glVector3(p);
	glEnd();
	glPointSize(40);
	glBegin(GL_POINTS);
	glColor3d(0,1,1);	foreach(Vector3 p, debugPoints)		glVector3(p);
	glEnd();
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
}

void Graph::draw2D(int width, int height)
{
	glDisable(GL_DEPTH_TEST);

    // Embed structure graph to 2D screen plane
    if(!property["embeded2D"].toBool())
	{
        GraphEmbed::embed( this );
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
	foreach(Link e, edges)
	{
		Vector3 c1 = node_centers[nmap.key(e.n1)];
		Vector3 c2 = node_centers[nmap.key(e.n2)];
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
}

void Graph::saveToFile( QString fileName )
{
	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

	QTextStream out(&file);
	out << "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n<document>\n";

	// Save nodes
	foreach(Node * n, nodes)
	{
		out << "<node>\n";

		// Type and ID
		out << QString("\t<id>%1</id>\n").arg(n->id);
		out << QString("\t<type>%1</type>\n\n").arg(n->type());

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

		out << "\n</node>\n\n";
	}

	// Save edges
	foreach(Link e, edges)
	{
		out << "<edge>\n";
		out << QString("\t<id>%1</id>\n").arg(e.id);
		out << QString("\t<type>%1</type>\n\n").arg(e.type);
		out << QString("\t<n>%1</n>\n").arg(e.n1->id);
		out << QString("\t<n>%1</n>\n").arg(e.n2->id);
		out << QString("\t<coord>%1 %2 %3</coord>\n").arg(e.coord[0].x()).arg(e.coord[0].y()).arg(e.coord[0].z());
		out << QString("\t<coord>%1 %2 %3</coord>\n").arg(e.coord[1].x()).arg(e.coord[1].y()).arg(e.coord[1].z());
		out << "\n</edge>\n\n";
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
		if(node_type == CURVE)
			addNode( new Curve( NURBSCurve(ctrlPoints, ctrlWeights, degree, false, true), id) );
		else if(node_type == SHEET)
		{
			if(control_count.size() < 2) continue;

			std::vector< std::vector<Vec3d> > cp = std::vector< std::vector<Vec3d> > (control_count.last(), std::vector<Vec3d>(control_count.first()));
			std::vector< std::vector<Scalar> > cw = std::vector< std::vector<Scalar> > (control_count.last(), std::vector<Scalar>(control_count.first()));

			for(int v = 0; v < control_count.last(); v++){
				for(int u = 0; u < control_count.first(); u++){
					int idx = (v * control_count.first()) + u;
					cp[v][u] = ctrlPoints[idx];
					cw[v][u] = ctrlWeights[idx];
				}
			}

			addNode( new Sheet( NURBSRectangle(cp, cw, degree, degree, false, false, true, true), id ) );
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

		addEdge(getNode(n.at(0).toElement().text()), getNode(n.at(1).toElement().text()));
	}

	file.close();
}

void Graph::materialize( SurfaceMeshModel * m )
{
	QElapsedTimer timer; timer.start();

	cached_mesh.clear();

	QBox3D box = bbox();
	QVector3D b = bbox().size();
	Scalar avg = (b.x() + b.y() + b.z()) / 3.0;
	Scalar voxel_size = avg / 50;

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
			QBox3D triBox;
			foreach(std::vector<Vector3> segment, parts)
			{
				foreach(Vector3 p, segment) triBox.unite(p + half_voxel);
			}
			vox.addBox( triBox.minimum(), triBox.maximum() );
		}

		qDebug() << "Node built [" << n->id << "] " << nodoe_timer.elapsed() << " ms";
	}

	vox.end();

	qDebug() << "Voxels built " << timer.elapsed() << " ms";

	vox.buildMesh(m, cached_mesh);

	if(m) cached_mesh.clear();
}

SurfaceMeshTypes::Vector3 Structure::Graph::nodeIntersection( Node * n1, Node * n2 )
{
	Scalar r = 0.1 * qMin(n1->bbox().size().length(), n2->bbox().size().length());

	std::vector< std::vector<Vector3> > parts1 = n1->discretized(r);
	std::vector< std::vector<Vector3> > parts2 = n2->discretized(r);

	Scalar minDist = DBL_MAX;
	int minI = 0, minJ = 0;

	QMatrix4x4 ts;
	ts.scale(1.01);

	// Compare parts bounding boxes
	for(int i = 0; i < (int)parts1.size(); i++)
	{
		QBox3D part1(parts1[i].front(), parts1[i].back());
		foreach(Vector3 p, parts1[i]) part1.united(p);
		part1.transform(ts);

		for(int j = 0; j < (int)parts2.size(); j++)
		{
			QBox3D part2(parts2[j].front(), parts2[j].back());
			foreach(Vector3 p, parts2[j]) part2.united(p);
			part2.transform(ts);

			if(part1.intersects(part2))
			{
				double dist = -part1.intersected(part2).size().length();

				if(dist < minDist)
				{
					minI = i;
					minJ = j;
					minDist = dist;
				}
			}
		}
	}

	// Find the exact intersection point
	std::vector<Vector3> p1 = parts1[ minI ];
	std::vector<Vector3> p2 = parts2[ minJ ];
	int g1 = p1.size(), g2 = p2.size();

	Vector3 c1,c2;
	Scalar s,t;

	if(g1 == 2 && g2 == 2) ClosestPointSegments		(p1[0],p1[1],  p2[0],p2[1],       s, t, c1, c2);	// curve -- curve
	if(g1 == 2 && g2 == 3) ClosestSegmentTriangle	(p1[0],p1[1],  p2[0],p2[1],p2[2],       c1, c2);	// curve -- sheet
	if(g1 == 3 && g2 == 2) ClosestSegmentTriangle	(p2[0],p2[1],  p1[0],p1[1],p1[2],       c1, c2);	// sheet -- curve
	if(g1 == 3 && g2 == 3) TriTriIntersect			(p1[0],p1[1],p1[2],  p2[0],p2[1],p2[2], c1, c2);	// sheet -- sheet

	//debugPoints.push_back(c1);
	//debugPoints.push_back(c2);
	//foreach(Vector3 w, p1) debugPoints2.push_back(w);
	//foreach(Vector3 w, p2) debugPoints3.push_back(w);

	return (c1 + c2) / 2.0;
}
