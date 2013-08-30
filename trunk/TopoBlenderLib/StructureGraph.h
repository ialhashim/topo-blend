#pragma once

#include <Eigen/Core>

#include "StructureCurve.h"
#include "StructureSheet.h"

#include "DynamicVoxel.h"
using namespace DynamicVoxelLib;

// DEBUG:
#include "../CustomDrawObjects.h"

// For camera
#include "qglviewer/qglviewer.h"

typedef QVector< QVector<QString> > NodeGroups;
Q_DECLARE_METATYPE( NodeGroups )

namespace Structure{

typedef QPair<Node*,Node*> QPairNodes;
typedef QPair<Link*,Link*> QPairLink;
typedef QPair<Scalar, QPairLink> ScalarLinksPair;

struct Graph
{
	// Properties
    QVector<Node*> nodes;
    QVector<Link*> edges;
	NodeGroups groups;

    Eigen::AlignedBox3d bbox();
    QMap< QString, QVariant > property;
	Eigen::MatrixXd adjacency;
	QMap< QString, void* > misc;
	QString name();

	// Special nodes
	QVector<Node*> aux_nodes;
	Node* auxNode( QString auxNodeID );

	int valence(Node * n);
	
	// Constructors
	Graph();
	Graph(QString fileName);
	Graph(const Graph & other);
    ~Graph();
	void init();
	
	// Modifiers
    Node * addNode( Node * n );
    Link * addEdge( Node * n1, Node * n2 );
	Link * addEdge( Node *n1, Node *n2, Array1D_Vector4d coord1, Array1D_Vector4d coord2, QString linkName = "" );
	Link * addEdge( QString n1_id, QString n2_id );
	int ueid;

    Node * removeNode( QString nodeID );
	void removeEdge( Node * n1, Node * n2 );
	void removeEdge( QString n1_id, QString n2_id );
	void removeEdges( QString nodeID );

    void addGroup(QVector<QString> nodes);
    void removeGroup(int groupIDX);
	QVector<QString> groupOf( QString nodeID );

	QString linkName( QString n1_id, QString n2_id );
    QString linkName( Node * n1, Node * n2 );

	// Node-wide Operations
	void setPropertyAll( QString prop_name, QVariant value );
	void setVisPropertyAll( QString prop_name, QVariant value );
	void setPropertyFor( QVector<QString> nodeIDs, QString prop_name, QVariant value );
	void setColorAll( QColor newNodesColor );
	
	QVector<Structure::Node*> nodesWithProperty( QString propertyName );
	QVector<Structure::Node*> nodesWithProperty( QString propertyName,  QVariant value );

	QVector<Structure::Node*> path(Structure::Node * from, Structure::Node * to);

	bool shareEdge( Node * n1, Node * n2 );

	void renameNode( QString oldNodeID, QString newNodeID );

    // Accessors
	Node* getNode(QString nodeID);
	Link* getEdge(int edgeUID);
	Link* getEdge(QString id1, QString id2);
	Vector3 nodeIntersection( Node * n1, Node * n2 );
	Curve* getCurve(Link * l);
	QVector<Link*> getEdges( QString nodeID );
	QMap< Link*, Array1D_Vector4d > linksCoords( QString nodeID );
	QVector<Link*> nodeEdges( QString nodeID );
	QVector<Node*> adjNodes( Node * node );
	QList<Link*> furthermostEdges( QString nodeID );
	Vector3 position( QString nodeID, Vector4d& coord );
	void replaceCoords( QString nodeA, QString nodeB, Array1D_Vector4d coordA, Array1D_Vector4d coordB );
	int indexOfNode( Node * node );

	// Input / Output
	void saveToFile(QString fileName) const;
	void loadFromFile(QString fileName);

	// Visualization
    void draw( QGLViewer * drawArea );
	void drawAABB();
    void draw2D(int width, int height);
	void drawNodeMeshNames( int & offSet );
	QImage fontImage;

	// [OLD] Synthesis
	void materialize( SurfaceMesh::Model * m, Scalar voxel_scaling = 1.0 );
	DynamicVoxel::QuadMesh cached_mesh;

    // Analysis
    Node * rootBySize();
    Node * rootByValence();
	int numCanVisit( Structure::Node * node );
	bool isConnected();
	bool isCutNode(QString nodeID);
	bool isBridgeEdge( Structure::Link * link );
	QVector< QVector<Node*> > split( QString nodeID );

	// Modifier
	void moveBottomCenterToOrigin();
	void normalize();
	void translate(Vector3 delta);
	void rotate(double angle, Vector3 axis);
	void scale(double scaleFactor);
	void transform(QMatrix4x4 mat);

	// Point Landmarks
	QVector<POINT_ID> selectedControlPointsByColor(QColor color);
	// DEBUG:
	std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
	VectorSoup vs,vs2,vs3;
	PolygonSoup ps,ps2,ps3;
	SphereSoup spheres, spheres2;
	void printAdjacency();
	void printLinksInfo();

	// Clean up
	void clearDebug();
	void clearAll();
	void clearSelections();
};

}

Q_DECLARE_METATYPE(SurfaceMesh::Model *)
Q_DECLARE_METATYPE(Structure::Graph *)
