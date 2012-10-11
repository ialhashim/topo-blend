#include <QFile>
#include <QDomElement>

#include "StructureGraph.h"
using namespace Structure;

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

	double scaling = 1.001;

	QBox3D intersection = n1->bbox(scaling).intersected(n2->bbox(scaling));
	Vector3 center = intersection.center();

	Vector3 c1 = n1->approxProjection(center);
	Vector3 c2 = n2->approxProjection(center);

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

void Graph::draw() const
{
    foreach(Node * n, nodes)
    {
        n->draw();
    }

    foreach(Link e, edges)
    {
        e.draw();
    }
}

void Graph::draw2D() const
{

}

void Structure::Graph::saveToFile( QString fileName )
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

void Structure::Graph::loadFromFile( QString fileName )
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
