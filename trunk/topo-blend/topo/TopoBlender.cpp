#include "TopoBlender.h"
using namespace Structure;

typedef QPair<Node*,Node*> QPairNodes;
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

	// while number of 'DONE' nodes is not equal to target
	while((flags = active.flags("state")).count(DONE) != target.nodes.size())
	{
		// Break if we ran out of 'ACTIVE' nodes
		if(flags.indexOf(ACTIVE,0) < 0) break;

		// Find an active node
		int active_idx = 0;
		foreach(int key, active.nodes.keys()){
			if(active.nodes[key].val("state") == ACTIVE){
				active_idx = key;
				break;
			}
		}

		SimpleNode & n_active = active.nodes[active_idx];
		QString activeNodeID = n_active.str("original");
		QString targetNodeID = n_active.str("correspond");

		// Get corresponding target node
		int target_idx = target.nodeIndex("original", targetNodeID);

		// Get coordinates of all links of both
		QMap<Link*, Vec2d> coord_active = g1->linksCoords(activeNodeID);
		QMap<Link*, Vec2d> coord_target = g2->linksCoords(targetNodeID);

		QMap<QPairLink, Scalar> dists;

		// Compute pair-wise distances [active coords] <-> [target coords]
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

		// Decide either remove links (when target has less links) or add links
		int linkDiff = coord_active.size() - coord_target.size();

		if(linkDiff > 0)
		{
			QVector<QPairLink> diffSet;
			QList< ScalarLinksPair > sortedDists = sortQMapByValue(dists);

			// Find excess links on active node
			for(int i = 0; i < linkDiff; i++)
			{
				// We take away from bottom of sorted list
				Link * link = sortedDists.takeLast().second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				// Remove the edges
				active.removeEdge(n_active.idx, other_idx);

				// Mark as 'DISCONNECTED'
				active.nodes[other_idx].set("state", DISCONNECTED);
			}

			// Propagate to remaining neighbors the 'ACTIVE' state
			foreach(ScalarLinksPair sp, sortedDists)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				// Mark as 'ACTIVE'
				active.nodes[other_idx].set("state", ACTIVE);

				// Set corresponding target node
				Link * linkTarget = sp.second.second;
				QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
				active.nodes[other_idx].set("correspond", otherNodeTarget);
			}
		}
		else
		{
			// link deficiency

		}

		// This node is now 'DONE'
		n_active.set("state", DONE);

		qDebug() << active.flags("state");

		break;
	}

    return blendedGraph;
}

void TopoBlender::bestPartialCorrespondence()
{
	QMultiMap<Scalar, QPairNodes> scores;

	/// 1) Compute scores for pairs
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

	/// 2) Find pair with minimal score
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

	/// 3) Assign "root" node as 'active'
	Node * root = minPairs.first().first;
	Node * targetRoot = minPairs.first().second;

	active = DynamicGraph(g1);

	active.flagNodes("state", SLEEP);
	active.getNode( root->id )->set("correspond", targetRoot->id);
	active.getNode( root->id )->set("state", ACTIVE);
}
