#pragma once

#include <Eigen/Core>

#include "StructureCurve.h"
#include "StructureSheet.h"

// DEBUG:
#include "../CustomDrawObjects.h"

// For OpenGL camera
#include "qglviewer/qglviewer.h"

namespace Structure{
	typedef QVector< QVector<QString> > NodeGroups;

	struct Graph
	{
		// Properties
		QVector<Node*> nodes;
		QVector<Link*> edges;
		NodeGroups groups;

		Eigen::AlignedBox3d bbox(bool isSkipUnready = false);
		Eigen::AlignedBox3d cached_bbox();

		PropertyMap property;
		int ueid;
		QString name();

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

		Node * removeNode( QString nodeID );
		void removeEdge( int uid );
		void removeEdge( Node * n1, Node * n2 );
		void removeEdge( QString n1_id, QString n2_id );
		void removeEdges( QString nodeID );

		void addGroup(QVector<QString> nodes);
		void removeGroup(int groupIDX);
		void removeGroup(QVector<QString> groupElements);
		QVector< QVector<QString> > groupsOf( QString nodeID );

		QString linkName( QString n1_id, QString n2_id );
		QString linkName( Node * n1, Node * n2 );

		// Node-wide Operations
		void setPropertyAll( QString prop_name, QVariant value );
		void setVisPropertyAll( QString prop_name, QVariant value );
		void setPropertyFor( QVector<QString> nodeIDs, QString prop_name, QVariant value );
		void setColorAll( QColor newNodesColor );
		void setColorFor( QString nodeID, QColor newColor );
	
		QVector<Structure::Node*> nodesWithProperty( QString propertyName );
		QVector<Structure::Node*> nodesWithProperty( QString propertyName,  QVariant value );

		QVector<Structure::Node*> path(Structure::Node * from, Structure::Node * to);

		bool shareEdge( Node * n1, Node * n2 );
		bool shareEdge( QString nid1, QString nid2 );
		void renameNode( QString oldNodeID, QString newNodeID );

		// Accessors
		Node* getNode(QString nodeID);
		Link* getEdge(int edgeUID);
		Link* getEdge(QString id1, QString id2);
		Curve* getCurve(Link * l);
		QVector<Link*> getEdges( QString nodeID );
		QMap< Link*, Array1D_Vector4d > linksCoords( QString nodeID );
		QVector<Link*> nodeEdges( QString nodeID );
		QVector<Node*> adjNodes( Node * node );
		void replaceCoords( QString nodeA, QString nodeB, Array1D_Vector4d coordA, Array1D_Vector4d coordB );
		int indexOfNode( Node * node );
		SurfaceMesh::Model* getMesh( QString nodeID );
		QList<Link*> furthermostEdges( QString nodeID );
		Vector3 position( QString nodeID, Vector4d& coord );
		Vector3 nodeIntersection( Node * n1, Node * n2 );

		// Input / Output
		void saveToFile(QString fileName) const;
		void loadFromFile(QString fileName);

		// TopoBlend related
		static Structure::Graph * actualGraph(Structure::Graph * fromGraph);

		// Visualization
		void draw( QGLViewer * drawArea = 0 );
		void drawAABB();
		void draw2D(int width, int height);
		void drawNodeMeshNames( int & offSet );
		QImage fontImage;

		// Analysis
		QList<QString> nodesCanVisit( Node * node );
		int numCanVisit( Structure::Node * node );

		bool isConnected();
		bool isCutNode( QString nodeID );
		bool isInCutGroup( QString nodeID );
		bool isBridgeEdge( Structure::Link * link );
		QVector< QVector<Node*> > split( QString nodeID );

		QVector<Node*> articulationPoints();
		void articulationDFS( int & cnt, int u, int v, QVector<int> & low, QVector<int> & pre );

		QVector<Node*> leaves();

		Node * rootBySize();
		Node * rootByValence();

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
		QMap< QString, void* > misc;

		// Clean up
		void clearDebug();
		void clearAll();
		void clearSelections();
	};
}

Q_DECLARE_METATYPE( QSharedPointer<SurfaceMeshModel> )
Q_DECLARE_METATYPE( Structure::Graph * )
Q_DECLARE_METATYPE( Structure::NodeGroups )
