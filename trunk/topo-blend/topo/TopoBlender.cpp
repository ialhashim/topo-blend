#include <QFileSystemModel>

#include "TopoBlender.h"
using namespace Structure;

#include "ExportDynamicGraph.h"

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

	// Clean up past work
	QDir directory(QDir::currentPath());
	QStringList filesList = directory.entryList(QDir::Files);
	foreach(QString f, filesList) 
	{
		if(f.endsWith(".png") || f.endsWith(".gv") || f.endsWith(".graphml")) 
			directory.remove(f);
	}

	int step = 0;
	QString graphCaption = QString("step%1").arg(step);
	QString graphSubtitle = QString("Initial graph");
	toGraphML(active, graphCaption);
	toGraphviz(active, graphCaption, true, graphCaption, graphSubtitle);

	// while number of 'DONE' nodes is not equal to target
	while((flags = active.flags("state")).count(DONE) != target.nodes.size())
	{
		QString log;

		// Break if we ran out of 'ACTIVE' nodes
		if(flags.indexOf(ACTIVE,0) < 0) 
			break;

		// Find an active node
		int active_idx = active.nodeIndex("state", ACTIVE);
		if(active_idx < 0) break;

		// Get corresponding target node
		SimpleNode & n_active = active.nodes[active_idx];
		QString activeNodeID = n_active.str("original");
		QString targetNodeID = n_active.str("correspond");
		int target_idx = target.nodeIndex("original", targetNodeID);

		// Get coordinates of all links of both
		QMap<Link*, Vec4d> coord_active = g1->linksCoords(activeNodeID);
		QMap<Link*, Vec4d> coord_target = g2->linksCoords(targetNodeID);

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

		QList< ScalarLinksPair > sortedDists = sortQMapByValue(dists);

		// Decide either remove links (when target has less links) or add links
		int linkDiff = coord_active.size() - coord_target.size();

		// Link abundance:
		if(linkDiff > 0)
		{
			// Find excess links on active node
			for(int i = 0; i < linkDiff; i++)
			{
				// We take away from bottom of sorted list
				Link * link = sortedDists.takeLast().second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);

				// Remove the edges
				active.removeEdge(n_active.idx, other_idx);

				// Mark as 'DISCONNECTED' if it is isolated
				if(active.valence(other_idx) == 0)
					active.nodes[other_idx].set("state", DISCONNECTED);
			}

			// To remaining neighbors, propagate 'ACTIVE' state
			foreach(ScalarLinksPair sp, sortedDists)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);
				int otherState = active.nodes[other_idx].val("state");

				// Mark as 'ACTIVE'
				if(otherState != DONE && otherState != DISCONNECTED)
					active.nodes[other_idx].set("state", ACTIVE);

				// Set corresponding target node
				Link * linkTarget = sp.second.second;
				QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
				active.nodes[other_idx].set("correspond", otherNodeTarget);
			}

			// Log activity
			log += QString("Removed links from node");
		}

		// link deficiency:
		if(linkDiff < 0)
		{
			// Add special links that need to be filled with [something]
			//	[something] nearby nodes ? 

			// propagate 'ACTIVE' state to neighbors
			foreach(ScalarLinksPair sp, sortedDists)
			{
				Link * link = sp.second.first;
				QString otherNodeID = link->otherNode( activeNodeID )->id;
				int other_idx = active.nodeIndex("original", otherNodeID);
				int otherState = active.nodes[other_idx].val("state");

				// Mark as 'ACTIVE'
				if(otherState != DONE && otherState != DISCONNECTED)
				{
					active.nodes[other_idx].set("state", ACTIVE);

					// Set corresponding target node
					Link * linkTarget = sp.second.second;
					QString otherNodeTarget = linkTarget->otherNode(targetNodeID)->id;
					active.nodes[other_idx].set("correspond", otherNodeTarget);
				}
			}

			log += QString("Need to add links to node");
		}

		// Same links, need to check quality of these links
		if(linkDiff == 0)
		{
			log += QString("Same links count");
		}

		// This node is now 'DONE'
		n_active.set("state", DONE);

		step++;

		QString graphCaption = QString("step%1").arg(step);
		toGraphML(active, graphCaption);
		toGraphviz(active, graphCaption, true, graphCaption, log);
	}

	graphCaption = QString("step%1").arg(++step);
	graphSubtitle = QString("Final graph");
	toGraphML(active, graphCaption);
	toGraphviz(active, graphCaption, true, graphCaption, graphSubtitle);

	// Create animated GIF (assuming ImageMagick installed)
	system(qPrintable( QString("convert -resize 800x800	-delay %1 -loop 0 *.png steps.gif").arg( 200 ) ));

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
