#include "Dijkstra.h"
#include "GraphDistance.h"
using namespace Structure;

GraphDistance::GraphDistance(Structure::Graph *graph)
{
    this->g = graph;
}

void GraphDistance::computeDistances( Structure::Node * startNode, const Vec4d & coords )
{
	foreach(Node * node, g->nodes)
	{
		if(node->type() == Structure::SHEET)
		{
			Sheet * n = (Sheet*)node;
			
			std::vector<Vector3> cpnts = n->controlPoints();

			adjacency_list_t adjacency_list( cpnts.size() );
			std::vector<weight_t> min_distance;
			std::vector<vertex_t> previous;

			int numU = n->surface.mNumUCtrlPoints, numV = n->surface.mNumVCtrlPoints;

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
							int ni = qRanged(0, u + i, numU-1);
							int nj = qRanged(0, v + j, numV-1);
							adj.insert(ni*numV + nj);
						}
					}
					adj.erase(idx);

					// Add its neighbors
					foreach(int nei, adj){
						double weight = (cpnts[idx] - cpnts[nei]).norm();
						adjacency_list[idx].push_back(neighbor(nei, weight));
					}
				}
			}

			int start_point = 0;
			
			// Compute distances
			DijkstraComputePaths(start_point, adjacency_list, min_distance, previous);

			double max_dist = -DBL_MAX;

			for(int i = 0; i < (int)cpnts.size(); i++)
			{
				test_pnt.push_back(cpnts[i]);
				test_dist.push_back(min_distance[i]);

				max_dist = qMax(max_dist, min_distance[i]);
			}

			// Normalize
			for(int i = 0; i < (int)cpnts.size(); i++)
				test_dist[i] /= max_dist;

			break;
		}

		foreach(Vector3 p, node->controlPoints())
		{
			dbgPoint(p); 
		}
	}
}

void GraphDistance::draw()
{
	int pointSize = 10;
	int i = 0;

	glDisable(GL_LIGHTING);
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
	glEnable(GL_LIGHTING);

	// Test
	glDisable(GL_LIGHTING);
	glPointSize(pointSize + 5);
	glBegin(GL_POINTS);
	for(int i = 0; i < (int)test_pnt.size(); i++)
	{
		glColorQt( qtJetColorMap(test_dist[i]) );
		glVector3( test_pnt[i] );
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
