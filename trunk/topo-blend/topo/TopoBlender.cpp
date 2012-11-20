#include "TopoBlender.h"
using namespace Structure;

typedef QPair<Link*,Link*> QPairLink;
typedef QPair<Scalar, QPairLink> ScalarLinksPair;

TopoBlender::TopoBlender(Graph *graph1, Graph *graph2, QObject *parent) : QObject(parent)
{
    g1 = graph1;
    g2 = graph2;

	source = DynamicGraph(g1);
	target = DynamicGraph(g2);
}

Graph TopoBlender::blend(Scalar t)
{
	Graph blendedGraph;

	/// 1) Best partial correspondence
	bestPartialCorrespondence();

	/// 2) BFS solve for Link discrepancy
	Flags flags;

	while((flags = active.flags("state")).count(DONE) != target.nodes.size())
	{
		// Find active node index and its corresponding node at target
		int active_idx = flags.indexOf(ACTIVE,0);
		if(active_idx < 0) continue;
		SimpleNode & n = active.nodes[active_idx];
		int target_idx = target.nodeIndex("original", n.str("correspond"));

		QMap<Link*, Vec2d> coord_active = g1->linksCoords(n.str("original"));
		QMap<Link*, Vec2d> coord_target = g2->linksCoords(target.nodes[target_idx].str("original"));

		QMap<QPairLink, Scalar> dists;

		foreach(Link * i, coord_active.keys())
		{
			double minDist = DBL_MAX;
			Link * closestLink = NULL;

			foreach(Link * j, coord_target.keys())
			{
				double dist = (coord_active[i] - coord_target[j]).norm();

				if(dist < minDist)
				{
					minDist = dist;
					closestLink = j;
				}
			}

			dists[qMakePair(i, closestLink)] = minDist;
		}

		int linkDiff = coord_active.size() - coord_target.size();

		if(linkDiff > 0)
		{
			QVector<QPairLink> diffSet;
			QList< ScalarLinksPair > sortedDists = sortQMapByValue(dists);

			// Excess links on active node
			for(int i = 0; i < linkDiff; i++)
			{
				Link * link = sortedDists.takeLast().second.first;
				QString otherNodeID = link->otherNode( n.str("original") )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				// Remove the edges
				active.removeEdge(n.idx, other_idx);
			}

			// Assign neighbors as 'active'
			foreach(ScalarLinksPair sp, sortedDists)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( n.str("original") )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				active.nodes[other_idx].set("state", ACTIVE);
			}
		}
		else
		{
			// link deficiency

		}

		n.set("state", DONE);

		qDebug() << active.flags("state");

		break;
	}

    return blendedGraph;
}

void TopoBlender::bestPartialCorrespondence()
{
	typedef QPair<Node*,Node*> QPairNodes;

	QMultiMap< Scalar, QPairNodes> scores;

	double maxScore = -DBL_MAX;

	// Compute scores for pairs
	foreach(Node * i, g1->nodes)
	{
		double areaI = i->area();
		Vector3 centerI = i->center();

		foreach(Node * j, g2->nodes)
		{
			// Disabled for now
			if(i->type() != j->type()) continue;

			double score = 0;

			double areaJ = j->area();

			// Score computation
			score += abs( areaI - areaJ );
			score += abs( (centerI - j->center()).norm() );
			score += 1.0 / qMin(areaI, areaJ);
			//score += abs( g1->valence(i) - g2->valence(j) );

			scores.insert(score, qMakePair(i,j));
		}
	}

	QList<QPairNodes> minPairs;
	double minScore = DBL_MAX;

	foreach(double curScore, scores.keys())
	{
		if(curScore < minScore)
		{
			minScore = curScore;
			minPairs = scores.values(minScore);
		}
	}

	// Assign "root" node
	Node * root = minPairs.first().first;
	Node * targetRoot = minPairs.first().second;

	active = DynamicGraph(g1);

	active.flagNodes("state", SLEEP);
	active.getNode( root->id )->set("state", ACTIVE);
	active.getNode( root->id )->set("correspond", targetRoot->id);
}
