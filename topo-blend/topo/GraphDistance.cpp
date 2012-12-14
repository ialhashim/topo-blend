#include "GraphDistance.h"
using namespace Structure;

GraphDistance::GraphDistance(Structure::Graph *graph)
{
    this->g = graph;
	this->isReady = false;
}

void GraphDistance::computeDistances( Structure::Node * startNode, const Vec4d & coords, double resolution )
{
	int gid = 0;

	// Setup control points adjacency
	foreach(Node * node, g->nodes)
	{
		int i = 0;

		nodesMap[node] = std::vector<GraphDistanceNode>();
		samplePoints[node] = std::vector<Vector3>();

		std::vector<Vector3> pointList;
		std::vector< std::vector<Vector3> > discretization = node->discretizedPoints( resolution );
		nodeCount[node] = std::make_pair(discretization.size(), discretization.front().size());

		foreach(std::vector<Vector3> pts, discretization){
			foreach(Vector3 p, pts){
				pointList.push_back(p);
			}
		}

		foreach(Vector3 p, pointList){
			samplePoints[node].push_back(p);
			allPoints.push_back(p);
			nodesMap[node].push_back( GraphDistanceNode(p, node, i++, gid++) );
			adjacency_list.push_back( std::vector<neighbor>() );
		}
	}

	// Compute neighbors and distances at each node
	foreach(Node * node, g->nodes)
	{			
		int gid = nodesMap[node].front().gid;

		if(node->type() == Structure::CURVE)
		{
			int N = nodesMap[node].size();

			for(int i = 0; i < N; i++)
			{
				std::set<int> adj;
				adj.insert(qRanged(0, i - 1, N - 1));
				adj.insert(qRanged(0, i + 1, N - 1));
				adj.erase(i);

				// Add neighbors
				foreach(int nei, adj){
					double weight = (samplePoints[node][i] - samplePoints[node][nei]).norm();
					adjacency_list[gid + i].push_back(neighbor(gid + nei, weight));
				}
			}
		}

		if(node->type() == Structure::SHEET)
		{
			Sheet * n = (Sheet*)node;

			int numU = nodeCount[n].first, numV = nodeCount[n].second;

			for(int u = 0; u < numU; u++)
			{
				for(int v = 0; v < numV; v++)
				{
					std::set<int> adj;

					int idx = u*numV + v;

					// Figure out the set of neighbors
					for(int i = -1; i <= 1; i++)
					{
						for(int j = -1; j <= 1; j++)
						{
							int ni = qRanged(0, u + i, numU - 1);
							int nj = qRanged(0, v + j, numV - 1);
							adj.insert(ni*numV + nj);
						}
					}
					adj.erase(idx);

					// Add its neighbors
					foreach(int nei, adj){
						double weight = (samplePoints[node][idx] - samplePoints[node][nei]).norm();
						adjacency_list[gid + idx].push_back(neighbor(gid + nei, weight));
					}
				}
			}
		}
	}

	// Connect between nodes
	foreach(Link e, g->edges)
	{
		int gid1 = nodesMap[e.n1].front().gid;
		int gid2 = nodesMap[e.n2].front().gid;

		// Get positions
		Vector3 pos1(0),pos2(0);
		e.n1->get(e.coord[0], pos1);
		e.n2->get(e.coord[1], pos2);

		QMap<double,int> dists1, dists2;

		// Find closest on node 1
		for(int i = 0; i < (int)samplePoints[e.n1].size(); i++) 
			dists1[(pos1-samplePoints[e.n1][i]).norm()] = i;
		int id1 = dists1.values().first();

		// Find closest on node 2
		for(int i = 0; i < (int)samplePoints[e.n2].size(); i++) 
			dists2[(pos2-samplePoints[e.n2][i]).norm()] = i;
		int id2 = dists2.values().first();

		// Connect them
		double weight = (samplePoints[e.n1][id1] - samplePoints[e.n2][id2]).norm();
		adjacency_list[gid1 + id1].push_back(neighbor(gid2 + id2, weight));
		adjacency_list[gid2 + id2].push_back(neighbor(gid1 + id1, weight));
	}

	// Test starting from point 0
	int start_point = 0;

	std::vector<weight_t> min_distance;
	std::vector<vertex_t> previous;

	// Compute distances
	DijkstraComputePaths(start_point, adjacency_list, min_distance, previous);

	double max_dist = -DBL_MAX;

	for(int i = 0; i < (int)allPoints.size(); i++)
	{
		max_dist = qMax(max_dist, min_distance[i]);
	}

	// Normalize
	for(int i = 0; i < (int)allPoints.size(); i++)
	{
		dists.push_back(min_distance[i] / max_dist);
	}

	isReady = true;
}

void GraphDistance::draw()
{
	if(!isReady) return;

	glDisable(GL_LIGHTING);

	int pointSize = 10;
	int i = 0;

	foreach(std::vector<Vec3d> pnts, debugPoints)
	{
		// Size
		pointSize += 2;
		glPointSize(pointSize);

		// Color
		QColor color = QColor(i < (int)debugColors.size() ? debugColors[i++] : qRandomColor());
		glColorQt(color);

		// Geometry
		glBegin(GL_POINTS);
		foreach(Vec3d p, pnts) glVector3(p);
		glEnd();
	}

	// Draw edges
	/*glLineWidth(4);
	glBegin(GL_LINES);
	int v1 = 0;
	foreach(std::vector<neighbor> adj, adjacency_list)
	{
		foreach(neighbor n, adj)
		{
			int v2 = n.target;

			glVector3(allPoints[v1]);
			glVector3(allPoints[v2]);
		}
		v1++;
	}
	glEnd();*/

	// Visualize resulting distances
	glPointSize(pointSize + 15);
	glBegin(GL_POINTS);
	for(int i = 0; i < (int)allPoints.size(); i++)
	{
		glColorQt( qtJetColorMap(dists[i]) );
		glVector3( allPoints[i] );
	}
	glEnd();

	glEnable(GL_LIGHTING);
}

void GraphDistance::dbgPoint( const Vec3d & p ){
	if(debugPoints.size() < 1) 
	{
		debugPoints.push_back(std::vector<Vec3d>());
		debugColors.push_back(qRandomColor());
	}
	debugPoints.back().push_back(p);
}

void GraphDistance::pushDebugPoints( const std::vector<Vec3d> & pnts ){
	debugPoints.push_back(pnts);
}
