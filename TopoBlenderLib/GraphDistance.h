#pragma once
#include "StructureGraph.h"
#include "Dijkstra.h"

typedef std::map< int, std::pair<int,double> > CloseMap;

// Assuming normalized geometry
extern double DIST_RESOLUTION;

struct GraphDistanceNode{
	Vector3 pos;
	Structure::Node * n;
	int idx, gid;
	GraphDistanceNode(Vector3 & position, Structure::Node * node, int index, int globalID):
		pos(position),n(node),idx(index),gid(globalID){}
};

typedef QPair<QString, Vector4d> PathPoint;

class GraphDistance
{
public:
    GraphDistance(Structure::Graph * graph, QVector<QString> exclude_nodes = QVector<QString>(), QVector<QString> exclude_edges = QVector<QString>());
	GraphDistance(Structure::Node * n);

	int globalID;

	void prepareNodes( Scalar resolution, const std::vector<Vector3> & startingPoints, 
		CloseMap & closestStart, QVector<Structure::Node *> nodes );

	void computeDistances( Vector3 startingPoint, double resolution );
	void computeDistances( std::vector<Vector3> startingPoints, double resolution );

	Structure::Graph * g;
	double used_resolution;

	adjacency_list_t adjacency_list;
	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;

	std::map< Structure::Node *, std::vector<GraphDistanceNode> > nodesMap;
	std::map< Structure::Node *, std::vector<Vector3> > samplePoints;
	std::map< Structure::Node *, std::pair<int,int> > nodeCount;

	std::vector<Vector3> allPoints;
	QVector< QPair<QString,Vector4d> > allCoords;
	std::vector<double> dists;
	std::vector<Structure::Node *> correspond;
	std::set< std::pair<int,int> > jumpPoints;
	QVector<QString> excludeNodes, excludeEdges;

	bool isReady;

	// Jump free paths
	struct PathPointPair{
		PathPoint a,b;
		double wA,wB;
		PathPointPair(PathPoint& A = PathPoint("",Vector4d(0,0,0,0))){
			a = b = A;
			wA = 1.0; wB = 0.0;
		}
		PathPointPair(PathPoint& A, PathPoint& B, double alpha){
			a = A;
			b = B;
			alpha = qMax(0.0,qMin(alpha,1.0));
			wA = 1 - alpha; wB = alpha;
		}
		Vector3 position(Structure::Graph * graph){
			Vector3 posA = graph->position(a.first, a.second);
			Vector3 posB = graph->position(b.first, b.second);
			return (posA * wA) + (posB * wB);
		}
	};

	double pathTo( Vector3 point, std::vector<Vector3> & path );
	double pathCoordTo( Vector3 point, QVector< PathPoint > & path );
	double pathCoordTo( NodeCoord& relativePoint, QVector< QPair<QString, Vector4d> > & path );
	Structure::Node * closestNeighbourNode( Vector3 to, double resolution = 0.25 );
	double distance( Vector3 point );
	void clear();

	double smoothPathCoordTo( Vector3 point, QVector< PathPointPair > & smooth_path );
	double smoothPathCoordTo( NodeCoord& relativePoint, QVector< PathPointPair > & smooth_path );

	// Helpers
	void smoothPath( QVector< QPair<QString, Vector4d> > path, QVector< PathPointPair > & smooth_path );
	static Array1D_Vector3 positionalPath( Structure::Graph * graph, QVector< GraphDistance::PathPointPair > & from_path, int smoothingIters = 0 );
	
	// DEBUG:
	void draw();
};

static inline QVector<QString> SingleNode(const QString & nodeID){
	return QVector<QString>(1, nodeID);
}

Q_DECLARE_METATYPE( QVector< GraphDistance::PathPointPair > )
