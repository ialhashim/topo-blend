#include <set>
#include <QFileSystemModel>
#include <QDockWidget>
#include <QApplication>
#include <QMainWindow>
#include <QtConcurrentRun>

#include "TopoBlender.h"
using namespace Structure;

#include "ExportDynamicGraph.h"

#include "Task.h"
#include "Scheduler.h"
#include "SchedulerWidget.h"

// Temporary solution for output
#include "surface_mesh/IO.h"

typedef std::pair<QString, QString> PairQString;

TopoBlender::TopoBlender( Structure::Graph * graph1, Structure::Graph * graph2, GraphCorresponder * useCorresponder, Scheduler * useScheduler, QObject *parent ) : QObject(parent)
{
    sg = graph1;
    tg = graph2;
	scheduler = useScheduler;

	/// STEP 1) Compute correspondences and align mis-aligned nodes
	this->gcoor = useCorresponder;
	this->gcoor->computeCorrespondences();

	/// STEP 2) Generate super graphs
	generateSuperGraphs();

	/// STEP 3) Generate tasks 
	active = super_sg;
	generateTasks();

	/// STEP 4) Order and schedule the tasks
	scheduler->activeGraph = active;
	scheduler->sourceGraph = super_sg;
	scheduler->targetGraph = super_tg;

	qApp->setOverrideCursor(Qt::WaitCursor);
	scheduler->schedule();
	qApp->restoreOverrideCursor();

	// Visualization - color similar items
	foreach(Structure::Node * n, scheduler->sourceGraph->nodes)
	{
		QString corrId = n->property["correspond"].toString();
		Structure::Node * other = scheduler->targetGraph->getNode(corrId);
		if(other) n->vis_property["color"].setValue( other->vis_property["color"] );
	}

	// Visualize super graphs
	DynamicGraph sdg(scheduler->activeGraph);
	DynamicGraph tdg(scheduler->targetGraph);

	toGraphviz(sdg, "_sourceGraph", true, QString("V = %1, E = %2").arg(sdg.nodes.size()).arg(sdg.edges.size()), "super source graph");
	toGraphviz(tdg, "_targetGraph", true, QString("V = %1, E = %2").arg(tdg.nodes.size()).arg(tdg.edges.size()), "super target graph");

	// Show the scheduler window:
	SchedulerWidget * sw = new SchedulerWidget( scheduler );
	QDockWidget *dock = new QDockWidget("Scheduler");
	dock->setWidget(sw);
	QMainWindow * win = (QMainWindow *) qApp->activeWindow();
	win->addDockWidget(Qt::BottomDockWidgetArea, dock);

	this->connect( scheduler, SIGNAL(startBlend()), SLOT(executeBlend()) );
}

void TopoBlender::executeBlend()
{
	/// STEP 4) Execute the tasks
	QtConcurrent::run( scheduler, &Scheduler::executeAll ); //scheduler->executeAll();
}

void TopoBlender::drawDebug()
{

}

// Super graphs

bool TopoBlender::isExtraNode( Structure::Node *node )
{
	return node->property["correspond"].toString().contains("null");
}

bool TopoBlender::isExtraEdge( Structure::Link *link )
{
	return link->property["correspond"].toString().contains("null");
}

void TopoBlender::tagEdge( Structure::Link *link, QString tag )
{
	link->property[tag] = false;
}

bool TopoBlender::taggedEdge( Structure::Link *link, QString tag )
{
	return link->property.contains(tag);
}

bool TopoBlender::isCorrespondedEdge( Structure::Link *link )
{
	return link->property.contains("corresponded") && link->property["corresponded"].toBool();
}

bool TopoBlender::isShareCorrespondedNode( Structure::Link * slink, Structure::Link * tlink )
{
	QString tn1 = slink->n1->property["correspond"].toString();
	QString tn2 = slink->n2->property["correspond"].toString();

	bool has_n1 = tlink->hasNode( tn1 );
	bool has_n2 = tlink->hasNode( tn2 );

	return has_n1 || has_n2;
}

QVector<QString> TopoBlender::cloneGraphNode( Structure::Graph *g, QString nodeID, int N )
{
	Structure::Node * node = g->getNode(nodeID);

	// Clone the nodes
	QVector<QString> cloned_node_IDs;
	QVector<Structure::Node *> cloned_nodes;
	for (int i = 0; i < N; i++)
	{
		Structure::Node * n = node->clone();
		n->id = nodeID + "_" + QString::number(i);

		g->addNode(n);

		cloned_node_IDs.push_back(n->id);
		cloned_nodes.push_back(n);
	}

	// Link the cloned nodes
	int currCloned = 0;
	foreach(Structure::Link *link, g->getEdges(nodeID))
	{
		Structure::Node * other = link->otherNode(nodeID);
		QString otherID = other->id;

		if (isExtraNode(other))
		{
			if (currCloned >= N)
			{
				continue;
				qDebug() << "Warning: clone nodes error.";
			}

			// Link other to one of the cloned node
			QString clonedID = cloned_nodes[currCloned++]->id;
			Structure::Node * cnode = g->getNode(clonedID);

			LinkCoords c1 = link->getCoord(other->id);
			LinkCoords c2 = link->getCoordOther(other->id);
			g->addEdge(other, cnode, c1, c2, g->linkName(other,cnode));
		}
		else
		{
			// Link other to all cloned nodes
			foreach (Structure::Node* cnode, cloned_nodes)
			{
				LinkCoords c1 = link->getCoord(other->id);
				LinkCoords c2 = link->getCoordOther(other->id);
				g->addEdge(other, cnode, c1, c2, g->linkName(other,cnode));
			}
		}

		// Remove this link
		g->removeEdge(node, other);
	}

	// remove the original node
	g->removeNode(nodeID);

	return cloned_node_IDs;
}

void TopoBlender::correspondSuperNodes()
{
	// Add virtual corresponding nodes for missing nodes
	foreach(QString snodeID, gcoor->nonCorresSource())
	{
		Structure::Node *snode = super_sg->getNode(snodeID);
		Structure::Node *ctnode = snode->clone();
		super_tg->addNode(ctnode);

		ctnode->id = snodeID + "_null";
		superNodeCorr[snodeID] = ctnode->id;
	}

	foreach(QString tnodeID, gcoor->nonCorresTarget())
	{
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		Structure::Node *csnode = tnode->clone();
		super_sg->addNode(csnode);

		csnode->id = tnodeID + "_null";
		superNodeCorr[csnode->id] = tnodeID;

		csnode->property["isReady"] = false;
	}

	// Build node correspondence for corresponded nodes
	QVector< PairQString > core_pairs;
	foreach (PART_LANDMARK vec2vec, gcoor->correspondences)
	{
		QVector<QString> sNodes = vec2vec.first;
		QVector<QString> tNodes = vec2vec.second;

		int sN = sNodes.size();
		int tN = tNodes.size();
		Structure::Node * snode = super_sg->getNode(sNodes.front());
		Structure::Node * tnode = super_tg->getNode(tNodes.front());

		// 1-to-1
		if (sN == 1 && tN == 1)	superNodeCorr[snode->id] = tnode->id;

		// N-to-1
		if (sN > 1)	{
			QVector<QString> ctnodeIDs = cloneGraphNode(super_tg, tnode->id, sN);
			for (int i = 0; i < sN; i++) superNodeCorr[sNodes[i]] = ctnodeIDs[i];
		}

		// 1-to-N
		if (tN > 1)	{
			QVector<QString> csnodeIDs = cloneGraphNode(super_sg, snode->id, tN);
			for (int i = 0; i < tN; i++) superNodeCorr[csnodeIDs[i]] = tNodes[i];
		}
	}

	// Keep tack of missing / extra nodes
	QVector<QString> missingNodes, extraNodes;

	// Store correspondences in the graphs
	foreach(QString snode, superNodeCorr.keys())
	{
		QString tnode = superNodeCorr[snode];

		Structure::Node * sn = super_sg->getNode(snode);
		Structure::Node * tn = super_tg->getNode(tnode);

		// When does this happen?
		if(!tn || !sn){
			continue;
		}

		if(tnode.contains("null")) extraNodes.push_back( snode );
		if(snode.contains("null")) missingNodes.push_back( tnode );

		sn->property["correspond"] = tnode;
		tn->property["correspond"] = snode;
	}

	printf("");
}

void TopoBlender::correspondSuperEdges()
{
	// Links for extra nodes are mapped to null edges
	// Null edges actually doesn't exist for now
	// Extra source nodes: do nothing
	// Extra target nodes: add edges for the null source node after growing is done
	foreach(QString snodeID, superNodeCorr.keys())
	{
		Structure::Node *snode = super_sg->getNode(snodeID);
		if(!snode) continue;

		if ( isExtraNode(snode) )
		{
			foreach (Structure::Link *sl, super_sg->getEdges(snodeID))
			{
				superEdgeCorr[sl->id] = sl->id + "_null";
				tagEdge(sl, "corresponded");
			}

			// snode and tnode cannot be null at the same time
			continue;
		}

		QString tnodeID = superNodeCorr[snodeID];
		Structure::Node *tnode = super_tg->getNode(tnodeID);
		if(!tnode) continue;

		if ( isExtraNode(tnode) && !super_tg->isCutNode(tnode->id) )
		{
			foreach (Structure::Link *tl, super_tg->getEdges(tnodeID))
			{
				superEdgeCorr[tl->id + "_null"] = tl->id;
				tagEdge(tl, "corresponded");
			}
		}
	}


	// Both ends are corresponded -> for sure
	QMap<QString, QString>::Iterator begin = superNodeCorr.begin(), end = superNodeCorr.end();
	QMap<QString, QString>::Iterator itr1, itr2;
	for (itr1 = begin; itr1 != end; itr1 ++) {
		for (itr2 = itr1+1; itr2 != end; itr2++)
		{
			QString sn1 = itr1.key(), sn2 = itr2.key();
			QString tn1 = itr1.value(), tn2 = itr2.value();

			Structure::Link *slink = super_sg->getEdge(sn1, sn2);
			Structure::Link *tlink = super_tg->getEdge(tn1, tn2);

			if ( slink && tlink)	
			{
				// Assign one-to-one edge correspondence
				superEdgeCorr[slink->id] = tlink->id;

				// Mark
				slink->property["corresponded"] = true;
				tlink->property["corresponded"] = true;
			}
		}
	}

	// Two corresponded nodes have the same number of non-corresponded edges
	// Correspond them one by one
	for (itr1 = begin; itr1 != end; itr1 ++) 
	{
		QString snode = itr1.key();
		QString tnode = itr1.value();

		// Non-corresponded links
		QVector<Structure::Link *> slinks, tlinks;
		foreach(Structure::Link * l, super_sg->getEdges(snode))
			if (! taggedEdge(l, "corresponded")) slinks.push_back(l);
		foreach(Structure::Link * l, super_tg->getEdges(tnode))
			if (! taggedEdge(l, "corresponded")) tlinks.push_back(l);


		// Correspond them only if numbers of links are equal
		if (slinks.size() == tlinks.size())
		{
			for (int i = 0; i < slinks.size(); i++)
			{
				// Assign corresponding edge
				superEdgeCorr[slinks[i]->id] = tlinks[i]->id;

				// Mark
				tagEdge(slinks[i], "corresponded");
				tagEdge(tlinks[i], "corresponded");
			}
		}
	}

	// Pool of non-corresponded edges on source & target
	QVector<Structure::Link *> nonCorrEdges, tnonCorrEdges;
	foreach (Structure::Link *sl, super_sg->edges){
		if (!isCorrespondedEdge(sl)) nonCorrEdges.push_back(sl);
	}
	foreach (Structure::Link *tl, super_tg->edges){
		if (!isCorrespondedEdge(tl)) tnonCorrEdges.push_back(tl);
	}

	// Find unique links on target
	QMap<Structure::Link*, int> slinkCount = linkCounts( nonCorrEdges, tnonCorrEdges, super_tg );
	QMap<Structure::Link*, int> tlinkCount = linkCounts( tnonCorrEdges, nonCorrEdges, super_sg );

	/// CASE: Shrink a cut-node
	if( tlinkCount.size() )
	{
		nonCorrEdges.clear();
		foreach(Structure::Link * sl, tlinkCount.keys()){
			if(tlinkCount[sl] == 1) nonCorrEdges.push_back(sl);
		}

		// Match similar links
		foreach(Structure::Link * tlink, tnonCorrEdges){
			foreach(Structure::Link * slink, nonCorrEdges){
				if( isShareCorrespondedNode(slink, tlink) ){
					// Assign corresponding edge
					superEdgeCorr[slink->id] = tlink->id;
					slink->property["corresponded"] = true;
					tlink->property["corresponded"] = true;
					break;
				}
			}
		}

		// Remaining edges
		nonCorrEdges.clear();
		foreach (Structure::Link *sl, super_sg->edges){
			if (!isCorrespondedEdge(sl)) nonCorrEdges.push_back(sl);
		}

		// Add missing edges to source super graph
		foreach(Structure::Link * slink, nonCorrEdges)
		{
			Structure::Node * sn1 = slink->n1;
			Structure::Node * sn2 = slink->n2;
			LinkCoords c1 = slink->getCoord(sn1->id);
			LinkCoords c2 = slink->getCoordOther(sn1->id);

			Structure::Node * tn1 = super_tg->getNode(sn1->property["correspond"].toString());
			Structure::Node * tn2 = super_tg->getNode(sn2->property["correspond"].toString());

			Structure::Link * tlink = super_tg->addEdge(tn1, tn2, c1, c2, super_tg->linkName(tn1, tn2));

			// Assign corresponding edge
			superEdgeCorr[slink->id] = tlink->id;

			slink->property["corresponded"] = true;
			tlink->property["corresponded"] = true;
		}
	}

	/// CASE: Grow a cut-node case
	if( slinkCount.size() )
	{
		tnonCorrEdges.clear();
		foreach(Structure::Link * tl, slinkCount.keys()){
			if(slinkCount[tl] == 1) tnonCorrEdges.push_back(tl);
		}

		// Match similar links
		foreach(Structure::Link * slink, nonCorrEdges){
			foreach(Structure::Link * tlink, tnonCorrEdges){
				if( isShareCorrespondedNode(slink, tlink) ){
					// Assign corresponding edge
					superEdgeCorr[slink->id] = tlink->id;
					slink->property["corresponded"] = true;
					tlink->property["corresponded"] = true;
					break;
				}
			}
		}

		// Remaining edges
		tnonCorrEdges.clear();
		foreach (Structure::Link *tl, super_tg->edges){
			if (!isCorrespondedEdge(tl)) tnonCorrEdges.push_back(tl);
		}

		// Add missing edges to source super graph
		foreach(Structure::Link * tlink, tnonCorrEdges)
		{
			Structure::Node * tn1 = tlink->n1;
			Structure::Node * tn2 = tlink->n2;
			LinkCoords c1 = tlink->getCoord(tn1->id);
			LinkCoords c2 = tlink->getCoordOther(tn1->id);

			Structure::Node * sn1 = super_sg->getNode(tn1->property["correspond"].toString());
			Structure::Node * sn2 = super_sg->getNode(tn2->property["correspond"].toString());

			Structure::Link * slink = super_sg->addEdge(sn1, sn2, c1, c2, super_sg->linkName(sn1, sn2));

			// Assign corresponding edge
			superEdgeCorr[slink->id] = tlink->id;

			slink->property["corresponded"] = true;
			tlink->property["corresponded"] = true;
		}
	}

	// Store correspondences in the graphs
	foreach (QString slinkID, superEdgeCorr.keys())
	{
		QString tlinkID = superEdgeCorr[slinkID];

		Structure::Link * slink = super_sg->getEdge(slinkID);
		Structure::Link * tlink = super_tg->getEdge(tlinkID);

		// Add missing links from extra nodes
		if(!slink) slink = addMissingLink(super_sg, tlink);
		if(!tlink) tlink = addMissingLink(super_tg, slink);

		slink->property["correspond"] = tlinkID;
		tlink->property["correspond"] = slinkID;
	}
}

QMap<Structure::Link*, int> TopoBlender::linkCounts(QVector<Structure::Link*> edgeGroupA, 
													QVector<Structure::Link*> edgeGroupB, 
													Structure::Graph * graphB)
{
	QMap<Structure::Link*, int> linkCount;

	foreach (Structure::Link *link, edgeGroupA){
		for(int i = 0; i < (int)edgeGroupB.size(); i++){
			for(int j = i + 1; j < (int)edgeGroupB.size(); j++)
			{
				Structure::Link * ti = edgeGroupB[i];
				Structure::Link * tj = edgeGroupB[j];

				QSet<QString> adjNodes;
				adjNodes.insert( graphB->getNode(ti->n1->id)->property["correspond"].toString() );
				adjNodes.insert( graphB->getNode(ti->n2->id)->property["correspond"].toString() );
				adjNodes.insert( graphB->getNode(tj->n1->id)->property["correspond"].toString() );
				adjNodes.insert( graphB->getNode(tj->n2->id)->property["correspond"].toString() );

				if( adjNodes.contains(link->n1->id) && adjNodes.contains(link->n2->id) )
				{
					linkCount[ti]++;
					linkCount[tj]++;
				}
			}
		}
	}

	return linkCount;
}

void TopoBlender::generateSuperGraphs()
{
	// Equalize resolution for corresponded nodes
	equalizeResolutions();

	// Two super graphs have one-to-one correspondence between nodes and edges
	// Two corresponded edge don't have to link two corresponded nodes
	super_sg = new Structure::Graph(*sg);
	super_tg = new Structure::Graph(*tg);

	// Correspond nodes in super graphs
	correspondSuperNodes();

	// Correspond edges in super graphs
	correspondSuperEdges();
}

void TopoBlender::generateTasks()
{
	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		Task * task;
		
		if (snodeID.contains("null"))  // Grow
			task = new Task( active, super_tg, Task::GROW, scheduler->tasks.size() );
		else if (tnodeID.contains("null")) // Shrink
			task = new Task( active, super_tg, Task::SHRINK, scheduler->tasks.size() );
		else
			task = new Task( active, super_tg, Task::MORPH, scheduler->tasks.size() );

		task->property["nodeID"] = snodeID;
		scheduler->tasks.push_back( task );
	}
}


void TopoBlender::equalizeResolutions()
{
	foreach (PART_LANDMARK vec2vec, gcoor->correspondences)
	{
		QVector<QString> sNodes = vec2vec.first;
		QVector<QString> tNodes = vec2vec.second;

		// Pick up the best one 
		Structure::Node * bestNode = NULL;
		int bestResolution = 0;

		foreach( QString sid, sNodes)
		{
			Structure::Node * snode = sg->getNode(sid);
			if (snode->numCtrlPnts() > bestResolution)
			{
				bestResolution = snode->numCtrlPnts();
				bestNode = snode;
			}
		}

		foreach( QString tid, tNodes)
		{
			Structure::Node * tnode = tg->getNode(tid);
			if (tnode->numCtrlPnts() > bestResolution)
			{
				bestResolution = tnode->numCtrlPnts();
				bestNode = tnode;
			}
		}

		// Equalize resolution to the best one
		foreach( QString sid, sNodes)
		{
			Structure::Node * snode = sg->getNode(sid);
			if (snode != bestNode) snode->equalizeControlPoints(bestNode);
		}

		foreach( QString tid, tNodes)
		{
			Structure::Node * tnode = tg->getNode(tid);
			if (tnode != bestNode) tnode->equalizeControlPoints(bestNode);
		}
	}
}

Structure::Link * TopoBlender::addMissingLink( Structure::Graph *g, Structure::Link * link )
{
	Structure::Node * bn1 = link->n1;
	Structure::Node * bn2 = link->n2;
	Structure::Node * an1 = g->getNode(bn1->property["correspond"].toString());
	Structure::Node * an2 = g->getNode(bn2->property["correspond"].toString());
	LinkCoords c1 = link->getCoord(bn1->id);
	LinkCoords c2 = link->getCoordOther(bn1->id);
	return g->addEdge(an1, an2, c1, c2, g->linkName(an1, an2));
}
